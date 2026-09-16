#include <Servo.h>

// TigerBall 500 mm BENCH controller
// Board: Teensy 3.2
// Driver: Sabertooth 2x12 in independent R/C mode, linear response,
//         loss-of-signal timeout enabled.
//
// Wiring:
//   Teensy pin 20 -> Sabertooth S1 (left)
//   Teensy pin 21 -> Sabertooth S2 (right)
//   Teensy GND    -> Sabertooth 0V
//   Sabertooth 5V -> NOT CONNECTED
//
// This is an open-loop, wheels-raised bench controller. The exact Sabertooth
// DIP configuration and accepted pulse endpoints must be checked on the
// physical board before the wheels touch the ground.

constexpr uint8_t LEFT_SIGNAL_PIN = 20;
constexpr uint8_t RIGHT_SIGNAL_PIN = 21;

constexpr int NEUTRAL_PULSE_US = 1500;
constexpr int PULSE_SPAN_US = 500;  // Proposed full range: 1000 to 2000 us.

// Change one value to -1 only if that wheel moves opposite to the requested
// direction. Establish this with the wheels raised and a 10% command.
constexpr int8_t LEFT_POLARITY = 1;
constexpr int8_t RIGHT_POLARITY = 1;

// The GUI refreshes a held command every 100 ms. If commands disappear, the
// Teensy commands neutral and disarms. This is a software timeout, not the
// final hardware-watchdog implementation required for animal use.
constexpr uint32_t COMMAND_TIMEOUT_MS = 400;

Servo leftOutput;
Servo rightOutput;

bool armed = false;
bool emergencyStopLatched = false;
uint32_t lastValidCommandMs = 0;
int leftPercent = 0;
int rightPercent = 0;

char usbBuffer[64];
char uartBuffer[64];
size_t usbLength = 0;
size_t uartLength = 0;

int percentToPulse(int percent, int polarity) {
  percent = constrain(percent, -100, 100);
  return NEUTRAL_PULSE_US + (percent * polarity * PULSE_SPAN_US) / 100;
}

void writeOutputs(int requestedLeft, int requestedRight) {
  leftPercent = constrain(requestedLeft, -100, 100);
  rightPercent = constrain(requestedRight, -100, 100);
  leftOutput.writeMicroseconds(percentToPulse(leftPercent, LEFT_POLARITY));
  rightOutput.writeMicroseconds(percentToPulse(rightPercent, RIGHT_POLARITY));
}

void neutralNow() {
  writeOutputs(0, 0);
}

void replyBoth(const char *message) {
  Serial.println(message);
  Serial2.println(message);
}

void handleLine(char *line) {
  while (*line == ' ' || *line == '\t') ++line;

  if (strcmp(line, "ARM") == 0) {
    neutralNow();
    if (emergencyStopLatched) {
      replyBoth("ERR,ESTOP_LATCHED");
      return;
    }
    armed = true;
    lastValidCommandMs = millis();
    replyBoth("OK,ARMED");
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    neutralNow();
    lastValidCommandMs = millis();
    replyBoth("OK,STOPPED");
    return;
  }

  if (strcmp(line, "DISARM") == 0) {
    neutralNow();
    armed = false;
    replyBoth("OK,DISARMED");
    return;
  }

  if (strcmp(line, "ESTOP") == 0) {
    neutralNow();
    armed = false;
    emergencyStopLatched = true;
    replyBoth("OK,ESTOP_LATCHED");
    return;
  }

  if (strcmp(line, "RESET_ESTOP") == 0) {
    neutralNow();
    armed = false;
    emergencyStopLatched = false;
    replyBoth("OK,ESTOP_RESET_DISARMED");
    return;
  }

  if (strcmp(line, "PING") == 0) {
    if (armed && !emergencyStopLatched) {
      lastValidCommandMs = millis();
      replyBoth("OK,PONG");
    } else {
      replyBoth("ERR,DISARMED");
    }
    return;
  }

  int requestedLeft = 0;
  int requestedRight = 0;
  if (sscanf(line, "MOVE,%d,%d", &requestedLeft, &requestedRight) == 2) {
    if (!armed || emergencyStopLatched) {
      neutralNow();
      replyBoth(emergencyStopLatched ? "ERR,ESTOP_LATCHED" : "ERR,DISARMED");
      return;
    }

    requestedLeft = constrain(requestedLeft, -100, 100);
    requestedRight = constrain(requestedRight, -100, 100);
    writeOutputs(requestedLeft, requestedRight);
    lastValidCommandMs = millis();

    char response[48];
    snprintf(response, sizeof(response), "OK,MOVE,%d,%d", requestedLeft, requestedRight);
    replyBoth(response);
    return;
  }

  neutralNow();
  replyBoth("ERR,BAD_COMMAND");
}

void servicePort(Stream &port, char *buffer, size_t &length, size_t capacity) {
  while (port.available() > 0) {
    const char incoming = static_cast<char>(port.read());

    if (incoming == '\n' || incoming == '\r') {
      if (length > 0) {
        buffer[length] = '\0';
        handleLine(buffer);
        length = 0;
      }
      continue;
    }

    if (length < capacity - 1) {
      buffer[length++] = incoming;
    } else {
      length = 0;
      neutralNow();
      armed = false;
      replyBoth("ERR,LINE_TOO_LONG_DISARMED");
    }
  }
}

void setup() {
  // Attach first and establish neutral before accepting any command.
  leftOutput.attach(LEFT_SIGNAL_PIN, NEUTRAL_PULSE_US - PULSE_SPAN_US,
                    NEUTRAL_PULSE_US + PULSE_SPAN_US);
  rightOutput.attach(RIGHT_SIGNAL_PIN, NEUTRAL_PULSE_US - PULSE_SPAN_US,
                     NEUTRAL_PULSE_US + PULSE_SPAN_US);
  neutralNow();

  Serial.begin(115200);   // Teensy USB serial: PC or Pi USB connection.
  Serial2.begin(115200);  // Pi GPIO UART: Teensy RX2 pin 9, TX2 pin 10.

  delay(500);
  neutralNow();
  replyBoth("READY,TIGERBALL_50CM_BENCH,DISARMED");
}

void loop() {
  servicePort(Serial, usbBuffer, usbLength, sizeof(usbBuffer));
  servicePort(Serial2, uartBuffer, uartLength, sizeof(uartBuffer));

  if (armed && millis() - lastValidCommandMs > COMMAND_TIMEOUT_MS) {
    neutralNow();
    armed = false;
    replyBoth("TIMEOUT,NEUTRAL_DISARMED");
  }

  delay(2);
}
