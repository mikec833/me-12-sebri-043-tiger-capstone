/*
 * teensy_bridge.ino -- minimal motor + encoder bridge.
 * Motor: Sabertooth 2x12 S1 input (R/C-style pulse, via Servo library).
 * Serial in:  "M,<speed>" (-1..1), or "S" to stop.
 * Serial out: "E,<ticks>,<ticks_per_s>,<millis>" every 100ms.
 * Motor auto-stops if no command arrives within 500ms.
 */
#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>

constexpr int ENC_A_PIN = 21;
constexpr int ENC_B_PIN = 22;
constexpr int S1_PIN = 5;
constexpr int PULSE_MIN_US     = 1000;
constexpr int PULSE_NEUTRAL_US = 1500;
constexpr int PULSE_MAX_US     = 2000;
constexpr unsigned long CMD_TIMEOUT_MS    = 500;
constexpr unsigned long ENCODER_STREAM_MS = 100;

Encoder encoder(ENC_A_PIN, ENC_B_PIN);
Servo motor;

float cmdSpeed = 0.0f;
unsigned long lastCmdMs = 0;
long lastCount = 0;
unsigned long lastStreamMs = 0;

void setMotor(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  motor.writeMicroseconds(PULSE_NEUTRAL_US + (int)(speed * (PULSE_MAX_US - PULSE_NEUTRAL_US)));
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.startsWith("M,")) {
    cmdSpeed = line.substring(2).toFloat();
    lastCmdMs = millis();
  } else if (line == "S") {
    cmdSpeed = 0.0f;
    lastCmdMs = millis();
  }
}

void setup() {
  Serial.begin(115200);
  motor.attach(S1_PIN);
  setMotor(0.0f);
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - lastStreamMs >= ENCODER_STREAM_MS) {
    long count = encoder.read();
    float ticksPerSec = (count - lastCount) / ((now - lastStreamMs) / 1000.0f);
    lastCount = count;
    lastStreamMs = now;
    Serial.print("E,");
    Serial.print(count);
    Serial.print(',');
    Serial.print(ticksPerSec, 2);
    Serial.print(',');
    Serial.print(now);
    Serial.print(',');
    Serial.print(digitalRead(ENC_A_PIN));
    Serial.print(',');
    Serial.println(digitalRead(ENC_B_PIN));
    
  }

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) cmdSpeed = 0.0f;
  setMotor(cmdSpeed);
}
