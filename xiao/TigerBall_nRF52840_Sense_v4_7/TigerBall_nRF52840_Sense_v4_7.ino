#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <Servo.h>

// Declared before any function: the Arduino builder inserts its generated
// prototypes above the first function, and several of them use this type.
enum CommandSource : uint8_t { SRC_NONE, SRC_BLE, SRC_SERIAL, SRC_BOTH };
 
// TigerBall 500 mm bench controller: XIAO nRF52840 -> Sabertooth 2x12.
// Arduino board package: Seeed nRF52 Boards (non-mbed).
// Use the Bluefruit and Servo libraries bundled with that board package.
// Sabertooth must be physically configured for independent R/C input mode.
//
// v4.7: motor commands from BLE (keeper website, unchanged) and from USB serial
// (Raspberry Pi), at the same time, with automatic arbitration:
//   * Both sources use the same command grammar and the same processCommand(),
//     so every v4.6 safety path (ramps, reversal gate, synchroniser, stall fault,
//     watchdog) applies to both.
//   * The source of the last accepted A owns control (controlOwner). Only the
//     owner can drive (M/I/B/P) or keep the watchdog alive. K, S and R work from
//     either source. A from BLE always wins (keeper override); A from serial is
//     refused while BLE owns an armed robot.
//   * Bluefruit callbacks only queue lines and set flags; all state changes
//     happen in loop().
//   * V replies VERSION,4.6 over BLE so the existing v4.6 website keeps
//     accepting this firmware, and VERSION,4.7 over serial.
// v4.6 is kept unchanged in encoder_bridge/ for rollback.

constexpr uint8_t LEFT_MOTOR_PIN = D10;   // Sabertooth S1
constexpr uint8_t RIGHT_MOTOR_PIN = D8;   // Sabertooth S2
constexpr uint8_t LEFT_ENCODER_A = D5;    // motor 1 yellow, through level shifter
constexpr uint8_t LEFT_ENCODER_B = D6;    // motor 1 white, through level shifter
constexpr uint8_t RIGHT_ENCODER_A = D3;   // motor 2 yellow, through level shifter
constexpr uint8_t RIGHT_ENCODER_B = D4;   // motor 2 white, through level shifter
constexpr uint8_t BATTERY_PIN = D0;        // switched pack + through 5.54k/979R divider
constexpr float BATTERY_R1_OHMS = 5540.0f;
constexpr float BATTERY_R2_OHMS = 979.0f;
constexpr float BATTERY_ADC_RANGE_V = 3.0f;  // AR_INTERNAL_3_0, not the 3.3 V GPIO rail
constexpr float BATTERY_ADC_MAX_COUNT = 4095.0f; // 12-bit ADC
constexpr uint32_t BATTERY_INTERVAL_MS = 1000;
constexpr uint32_t LED_TOGGLE_INTERVAL_MS = 100; // 100 ms on + 100 ms off = 5 Hz
constexpr float LOW_BATTERY_TRIP_V = 10.0f;
constexpr float LOW_BATTERY_CLEAR_V = 10.3f;

constexpr float batteryVoltageFromAdc(uint16_t count) {
  return count * BATTERY_ADC_RANGE_V / BATTERY_ADC_MAX_COUNT *
    (BATTERY_R1_OHMS + BATTERY_R2_OHMS) / BATTERY_R2_OHMS;
}
static_assert(batteryVoltageFromAdc(2650) > 12.9f &&
              batteryVoltageFromAdc(2650) < 13.0f,
              "Battery divider/ADC conversion does not match measured 979 ohm divider");
 
constexpr int NEUTRAL_US = 1500;
constexpr int FULL_REVERSE_US = 1000;
constexpr int FULL_FORWARD_US = 2000;
constexpr float OUTPUT_LIMIT_PERCENT = 100.0f;
constexpr uint32_t PIVOT_RESPONSE_MS = 100;
 
// Pololu #2825: 64 counts/motor revolution * 70:1 = 4480 counts/output rev
// when all four quadrature edges are counted.
constexpr float COUNTS_PER_OUTPUT_REV = 4480.0f;
constexpr uint32_t CONTROL_INTERVAL_MS = 20;
constexpr uint32_t SPEED_INTERVAL_MS = 100;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;
constexpr uint32_t SOFT_TIMEOUT_MS = 500;
constexpr uint32_t HARD_TIMEOUT_MS = 1200;
 
// Straight-line synchroniser. These are conservative starting values and must
// be tuned wheels-raised before loaded use.
constexpr float SYNC_MIN_COMMAND_PERCENT = 12.0f;
constexpr float SYNC_KP_PERCENT_PER_RPM = 0.18f;
constexpr float SYNC_KI_PERCENT_PER_RPM_SECOND = 0.04f;
constexpr float SYNC_MAX_CORRECTION_PERCENT = 10.0f;
constexpr float STALL_MOVING_RPM = 8.0f;
constexpr float STALL_STOPPED_RPM = 1.0f;
constexpr uint32_t STALL_TIME_MS = 800;
// A reversal waits for neutral PWM and two fresh, near-zero encoder samples.
// This is a best-effort software interlock, not motor-current protection.
constexpr float REVERSAL_STOP_RPM = 1.0f;
constexpr uint8_t REVERSAL_STOP_SAMPLES = 2;
 
const char *DEVICE_NAME = "TigerBall-50-XIAO";
const char *SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const char *RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
const char *TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
 
BLEService motorService(SERVICE_UUID);
BLECharacteristic rxCharacteristic(RX_UUID);
BLECharacteristic txCharacteristic(TX_UUID);
Servo leftMotor;
Servo rightMotor;
volatile bool bleConnected = false;

// ---------------- Command sources and arbitration (v4.7) ----------------
CommandSource controlOwner = SRC_NONE;  // source of the last accepted A

// Ramp and output limit as last sent by each source with C. A loads the
// arming source's values; every disarm resets both maxima to 10 %.
struct SourceConfig {
  uint32_t accelerationMs;
  uint32_t brakingMs;
  float maximumPercent;
};
SourceConfig sourceConfig[3] = {{800, 700, 10.0f}, {800, 700, 10.0f}, {800, 700, 10.0f}};

// BLE lines are assembled in the Bluefruit callback task and handed to loop()
// through this queue. Item = one NUL-terminated line of up to 80 characters.
constexpr size_t COMMAND_LINE_MAX = 80;
constexpr UBaseType_t BLE_QUEUE_DEPTH = 8;
constexpr char BLE_OVERFLOW_MARKER = '\x01';  // a line holding only this = overflow
QueueHandle_t bleLineQueue = nullptr;
char bleRxBuffer[COMMAND_LINE_MAX + 1];
size_t bleRxLength = 0;
volatile bool bleQueueFull = false;       // loop() treats this as S from BLE
volatile bool bleConnectEvent = false;
volatile bool bleDisconnectEvent = false;

// USB serial line reader (loop() only).
constexpr int SERIAL_READ_BUDGET = 64;    // bytes per loop() iteration
char serialRxBuffer[COMMAND_LINE_MAX + 1];
size_t serialRxLength = 0;
bool serialDiscarding = false;            // overflowed; skip to next terminator
uint32_t lastNotOwnerReplyMs = 0;

volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
volatile uint8_t leftPreviousState = 0;
volatile uint8_t rightPreviousState = 0;
 
float leftRpm = 0.0f;
float rightRpm = 0.0f;
float targetLeft = 0.0f;
float targetRight = 0.0f;
float requestedLeft = 0.0f;
float requestedRight = 0.0f;
float rampedLeft = 0.0f;
float rampedRight = 0.0f;
float appliedLeft = 0.0f;
float appliedRight = 0.0f;
bool requestedDirectPivot = false;
bool rampPivotMode = false;
float motionEnvelope = 0.0f;
bool waitingForNeutral = false;
uint8_t stoppedSamples = 0;
int8_t recentLeftSign = 0;
int8_t recentRightSign = 0;
float syncIntegral = 0.0f;
float syncCorrection = 0.0f;
uint32_t accelerationMs = 800;
uint32_t brakingMs = 700;
float configuredMaximumPercent = 10.0f;
uint32_t lastCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastSpeedMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastBatteryMs = 0;
uint32_t lastLedToggleMs = 0;
float batteryVoltage = 0.0f;
bool lowBattery = false;
bool redLedOn = false;
uint32_t stallStartedMs = 0;
bool armed = false;
bool stallFault = false;
 
constexpr int8_t QUADRATURE_TABLE[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};
 
float clampOutput(float value) {
  return constrain(value, -configuredMaximumPercent, configuredMaximumPercent);
}

int8_t commandSign(float value) {
  return value > 0.5f ? 1 : (value < -0.5f ? -1 : 0);
}
 
float signOf(float value) {
  return value < 0.0f ? -1.0f : 1.0f;
}
 
void attachMotorPwm() {
  // The nRF52 Servo library generates hardware PWM with a 20 ms period.
  leftMotor.attach(LEFT_MOTOR_PIN, FULL_REVERSE_US, FULL_FORWARD_US);
  rightMotor.attach(RIGHT_MOTOR_PIN, FULL_REVERSE_US, FULL_FORWARD_US);
}

void applyMotorOutputs(float leftPercent, float rightPercent) {
  // Keep command-mode changes from interleaving with the final output clamp.
  taskENTER_CRITICAL();
  leftPercent = clampOutput(leftPercent);
  rightPercent = clampOutput(rightPercent);
  const int leftUs = NEUTRAL_US + lroundf(leftPercent * 5.0f);
  const int rightUs = NEUTRAL_US + lroundf(rightPercent * 5.0f);
  leftMotor.writeMicroseconds(constrain(leftUs, FULL_REVERSE_US, FULL_FORWARD_US));
  rightMotor.writeMicroseconds(constrain(rightUs, FULL_REVERSE_US, FULL_FORWARD_US));
  appliedLeft = leftPercent;
  appliedRight = rightPercent;
  if (commandSign(leftPercent)) recentLeftSign = commandSign(leftPercent);
  if (commandSign(rightPercent)) recentRightSign = commandSign(rightPercent);
  taskEXIT_CRITICAL();
}
 
void resetSynchroniser() {
  syncIntegral = 0.0f;
  syncCorrection = 0.0f;
  stallStartedMs = 0;
}
 
// v4.6 neutralAndDisarm() body. Leaves controlOwner alone so A can disarm and
// re-arm under a new owner without broadcasting a spurious OWNER,NONE.
void disarmOutputs() {
  requestedLeft = 0.0f;
  requestedRight = 0.0f;
  requestedDirectPivot = false;
  waitingForNeutral = false;
  targetLeft = 0.0f;
  targetRight = 0.0f;
  rampedLeft = 0.0f;
  rampedRight = 0.0f;
  motionEnvelope = 0.0f;
  rampPivotMode = false;
  resetSynchroniser();
  applyMotorOutputs(0.0f, 0.0f);
  // A new arm needs a fresh C command; stale high-output settings do not
  // survive a disconnect, fault, STOP, or tab-change disarm. This applies to
  // the stored settings of both sources.
  configuredMaximumPercent = 10.0f;
  sourceConfig[SRC_BLE].maximumPercent = 10.0f;
  sourceConfig[SRC_SERIAL].maximumPercent = 10.0f;
  armed = false;
}

void notifyLine(const String &line) {
  if (!bleConnected || !txCharacteristic.notifyEnabled()) return;
  // Stay within the default 20-byte BLE notification payload. The website
  // reassembles newline-terminated messages across notifications.
  for (size_t offset = 0; offset < line.length(); offset += 20) {
    const size_t chunk = min(static_cast<size_t>(20), line.length() - offset);
    txCharacteristic.notify(line.c_str() + offset, chunk);
  }
}

// Serial writes are whole lines from loop() only, so they never interleave with
// MEAS. A line that does not fit the USB buffer is dropped, never blocked on.
void serialLine(const char *line) {
  const int length = strlen(line);
  if (length == 0 || !Serial || Serial.availableForWrite() < length) return;
  Serial.write(line, length);
}

void sendLine(CommandSource dest, const char *line) {
  if (dest == SRC_BLE || dest == SRC_BOTH) notifyLine(line);
  if (dest == SRC_SERIAL || dest == SRC_BOTH) serialLine(line);
}

const char *sourceName(CommandSource source) {
  return source == SRC_BLE ? "BLE" : (source == SRC_SERIAL ? "SERIAL" : "NONE");
}

void setOwner(CommandSource owner) {
  if (owner == controlOwner) return;
  controlOwner = owner;
  char message[24];
  snprintf(message, sizeof(message), "OWNER,%s\n", sourceName(owner));
  sendLine(SRC_BOTH, message);
}

void neutralAndDisarm() {
  disarmOutputs();
  setOwner(SRC_NONE);
}

void setTargets(float left, float right, bool directPivot) {
  if (!armed || stallFault) return;
  requestedLeft = clampOutput(left);
  requestedRight = clampOutput(right);
  requestedDirectPivot = directPivot;
  if (commandSign(requestedLeft) == 0 && commandSign(requestedRight) == 0) {
    waitingForNeutral = false;
    targetLeft = 0.0f;
    targetRight = 0.0f;
  } else if (waitingForNeutral) {
    // Latest request wins, but no new direction runs until both wheels stop.
    targetLeft = 0.0f;
    targetRight = 0.0f;
  } else {
    const bool leftReverses = commandSign(requestedLeft) && recentLeftSign &&
      commandSign(requestedLeft) != recentLeftSign;
    const bool rightReverses = commandSign(requestedRight) && recentRightSign &&
      commandSign(requestedRight) != recentRightSign;
    // Gate entry into a pivot from a moving non-pivot command, but do not
    // re-gate every 100 ms heartbeat after that same pivot has started.
    if (leftReverses || rightReverses ||
        (directPivot && !rampPivotMode && stoppedSamples < REVERSAL_STOP_SAMPLES &&
         (fabsf(leftRpm) > REVERSAL_STOP_RPM || fabsf(rightRpm) > REVERSAL_STOP_RPM))) {
      waitingForNeutral = true;
      stoppedSamples = 0;
      targetLeft = 0.0f;
      targetRight = 0.0f;
      sendLine(controlOwner, "STATE,WAIT_NEUTRAL\n");
    } else {
      targetLeft = requestedLeft;
      targetRight = requestedRight;
      rampPivotMode = directPivot;
    }
  }
  lastCommandMs = millis();
}
 
float stepEnvelope(float current, float target, float dtSeconds) {
  const bool slowing = target < current;
  const uint32_t requestedDuration = rampPivotMode ? PIVOT_RESPONSE_MS :
    (slowing ? brakingMs : accelerationMs);
  const float durationSeconds = requestedDuration / 1000.0f;
  // The selected full output, not 100%, takes the selected ramp duration.
  const float maximumStep = (configuredMaximumPercent / durationSeconds) * dtSeconds;
  if (target > current) return min(current + maximumStep, target);
  if (target < current) return max(current - maximumStep, target);
  return current;
}
 
void updateLeftEncoder() {
  const uint8_t state = (digitalRead(LEFT_ENCODER_A) << 1) | digitalRead(LEFT_ENCODER_B);
  const uint8_t index = (leftPreviousState << 2) | state;
  leftCount += QUADRATURE_TABLE[index];
  leftPreviousState = state;
}
 
void updateRightEncoder() {
  const uint8_t state = (digitalRead(RIGHT_ENCODER_A) << 1) | digitalRead(RIGHT_ENCODER_B);
  const uint8_t index = (rightPreviousState << 2) | state;
  rightCount += QUADRATURE_TABLE[index];
  rightPreviousState = state;
}
 
void updateSpeedEstimate(uint32_t now) {
  static int32_t previousLeftCount = 0;
  static int32_t previousRightCount = 0;
  const uint32_t elapsedMs = now - lastSpeedMs;
  if (elapsedMs < SPEED_INTERVAL_MS) return;
 
  int32_t currentLeftCount;
  int32_t currentRightCount;
  taskENTER_CRITICAL();
  currentLeftCount = leftCount;
  currentRightCount = rightCount;
  taskEXIT_CRITICAL();
 
  const float rpmFactor = 60000.0f / (COUNTS_PER_OUTPUT_REV * elapsedMs);
  leftRpm = (currentLeftCount - previousLeftCount) * rpmFactor;
  rightRpm = (currentRightCount - previousRightCount) * rpmFactor;
  previousLeftCount = currentLeftCount;
  previousRightCount = currentRightCount;
  lastSpeedMs = now;

  const bool neutralAndStopped =
    fabsf(rampedLeft) <= 0.5f && fabsf(rampedRight) <= 0.5f &&
    fabsf(appliedLeft) <= 0.5f && fabsf(appliedRight) <= 0.5f &&
    fabsf(leftRpm) <= REVERSAL_STOP_RPM &&
    fabsf(rightRpm) <= REVERSAL_STOP_RPM;
  stoppedSamples = neutralAndStopped ? min(static_cast<int>(stoppedSamples) + 1,
    static_cast<int>(REVERSAL_STOP_SAMPLES)) : 0;
  if (stoppedSamples >= REVERSAL_STOP_SAMPLES) {
    recentLeftSign = 0;
    recentRightSign = 0;
    if (waitingForNeutral) {
      waitingForNeutral = false;
      targetLeft = requestedLeft;
      targetRight = requestedRight;
      if (commandSign(targetLeft) || commandSign(targetRight))
        rampPivotMode = requestedDirectPivot;
      sendLine(controlOwner, "STATE,NEUTRAL_READY\n");
    }
  }
}
 
bool straightSynchronisationRequested() {
  if (!armed || stallFault) return false;
  if (targetLeft * targetRight <= 0.0f) return false;
  if (fabsf(targetLeft) < SYNC_MIN_COMMAND_PERCENT || fabsf(targetRight) < SYNC_MIN_COMMAND_PERCENT) return false;
  if (fabsf(rampedLeft) < SYNC_MIN_COMMAND_PERCENT || fabsf(rampedRight) < SYNC_MIN_COMMAND_PERCENT) return false;
  return fabsf(fabsf(targetLeft) - fabsf(targetRight)) <= 1.0f;
}
 
void updateSynchroniser(float dtSeconds, uint32_t now) {
  if (!straightSynchronisationRequested()) {
    resetSynchroniser();
    applyMotorOutputs(rampedLeft, rampedRight);
    return;
  }
 
  const float leftSpeed = fabsf(leftRpm);
  const float rightSpeed = fabsf(rightRpm);
  const float errorRpm = leftSpeed - rightSpeed;
  syncIntegral = constrain(syncIntegral + errorRpm * dtSeconds, -100.0f, 100.0f);
  const float correctionLimit = min(SYNC_MAX_CORRECTION_PERCENT,
    0.5f * min(fabsf(rampedLeft), fabsf(rampedRight)));
  syncCorrection = constrain(
    SYNC_KP_PERCENT_PER_RPM * errorRpm + SYNC_KI_PERCENT_PER_RPM_SECOND * syncIntegral,
    -correctionLimit,
    correctionLimit
  );
 
  const float correctedLeft = rampedLeft - signOf(rampedLeft) * syncCorrection;
  const float correctedRight = rampedRight + signOf(rampedRight) * syncCorrection;
  applyMotorOutputs(correctedLeft, correctedRight);
 
  const bool oneWheelStalled =
    (leftSpeed < STALL_STOPPED_RPM && rightSpeed > STALL_MOVING_RPM) ||
    (rightSpeed < STALL_STOPPED_RPM && leftSpeed > STALL_MOVING_RPM);
  const bool bothWheelsStopped =
    fabsf(rampedLeft) >= SYNC_MIN_COMMAND_PERCENT &&
    fabsf(rampedRight) >= SYNC_MIN_COMMAND_PERCENT &&
    leftSpeed < STALL_STOPPED_RPM && rightSpeed < STALL_STOPPED_RPM;
  if (oneWheelStalled || bothWheelsStopped) {
    if (stallStartedMs == 0) stallStartedMs = now;
    if (now - stallStartedMs >= STALL_TIME_MS) {
      stallFault = true;
      neutralAndDisarm();
      sendLine(SRC_BOTH, "FAULT,ENCODER_STALL\n");
    }
  } else {
    stallStartedMs = 0;
  }
}
 
void processCommand(String command, CommandSource src) {
  command.trim();
  if (command.length() == 0) return;
  const bool fromOwner = src == controlOwner;

  if (command == "V") {
    // The v4.6 website only enables Arm after an exact VERSION,4.6 reply.
    sendLine(src, src == SRC_BLE ? "VERSION,4.6\n" : "VERSION,4.7\n");
    return;
  }
 
  if (command == "A") {
    if (stallFault) {
      // v4.6 ignores A silently during a fault; the Pi is told why.
      if (src == SRC_SERIAL) sendLine(SRC_SERIAL, "FAULT,ENCODER_STALL\n");
      return;
    }
    if (src == SRC_SERIAL && armed && controlOwner == SRC_BLE) {
      sendLine(SRC_SERIAL, "STATE,BUSY,BLE\n");
      return;
    }
    // Copy before disarming: the disarm resets both stored maxima to 10 %.
    const SourceConfig armedConfig = sourceConfig[src];
    disarmOutputs();
    accelerationMs = armedConfig.accelerationMs;
    brakingMs = armedConfig.brakingMs;
    configuredMaximumPercent = armedConfig.maximumPercent;
    armed = true;
    lastCommandMs = millis();
    setOwner(src);
    sendLine(src, "STATE,ARMED\n");
    return;
  }
  if (command == "K") {
    neutralAndDisarm();
    sendLine(SRC_BOTH, "STATE,DISARMED\n");
    return;
  }
  if (command == "R") {
    neutralAndDisarm();
    stallFault = false;
    // FAULT is sent to both sources, so its reset is too.
    sendLine(SRC_BOTH, "STATE,FAULT_RESET\n");
    return;
  }
  if (command == "S") {
    if (armed) {
      requestedLeft = 0.0f;
      requestedRight = 0.0f;
      requestedDirectPivot = false;
      waitingForNeutral = false;
      targetLeft = 0.0f;
      targetRight = 0.0f;
      // A stop from the other source is honoured but never feeds the watchdog.
      if (fromOwner) lastCommandMs = millis();
    }
    return;
  }
 
  // This board package does not enable floating-point sscanf by default.
  if (command.length() < 3 || command[1] != ',') return;
  const char type = command[0];
  char *end = nullptr;
  const char *firstStart = command.c_str() + 2;
  const float first = strtof(firstStart, &end);
  if (end == firstStart || *end != ',') return;
  const char *secondStart = end + 1;
  const float second = strtof(secondStart, &end);
  if (end == secondStart || !isfinite(first) || !isfinite(second)) return;

  if (type == 'C') {
    if (*end != ',') return;
    const char *thirdStart = end + 1;
    const float third = strtof(thirdStart, &end);
    if (end == thirdStart || *end != '\0' || !isfinite(third) ||
        first < 100.0f || first > 4000.0f ||
        second < 100.0f || second > 4000.0f ||
        third < 10.0f || third > 100.0f ||
        floorf(first) != first || floorf(second) != second || floorf(third) != third) return;
    sourceConfig[src].accelerationMs = static_cast<uint32_t>(first);
    sourceConfig[src].brakingMs = static_cast<uint32_t>(second);
    sourceConfig[src].maximumPercent = third;
    if (armed && fromOwner) {
      // v4.6 applies C live while armed; only the owner may do that now.
      accelerationMs = sourceConfig[src].accelerationMs;
      brakingMs = sourceConfig[src].brakingMs;
      configuredMaximumPercent = third;
    }
    return;
  }
  if (*end != '\0') return;
  if (type == 'M' || type == 'I' || type == 'B' || type == 'P') {
    if (fabsf(first) > OUTPUT_LIMIT_PERCENT || fabsf(second) > OUTPUT_LIMIT_PERCENT) return;
    // P is a short-ramp opposite-wheel pivot; legacy I cannot bypass the gate.
    if (type == 'P' && (first * second >= 0.0f || fabsf(first + second) > 1.0f)) return;
    if (!armed || !fromOwner) {
      const uint32_t now = millis();
      if (src == SRC_SERIAL && now - lastNotOwnerReplyMs >= 1000) {
        lastNotOwnerReplyMs = now;
        sendLine(SRC_SERIAL, "STATE,NOT_OWNER\n");
      }
      return;
    }
    setTargets(first, second, type == 'P');
  }
}

// ---------------- BLE callbacks (Bluefruit task) ----------------
// These only assemble lines and set flags. processCommand() and every other
// state change run in loop().

void queueBleLine(const char *line) {
  char item[COMMAND_LINE_MAX + 1];
  strncpy(item, line, COMMAND_LINE_MAX);
  item[COMMAND_LINE_MAX] = '\0';
  if (xQueueSend(bleLineQueue, item, 0) != pdPASS) bleQueueFull = true;
}

void onRxWrite(uint16_t, BLECharacteristic *, uint8_t *data, uint16_t length) {
  bool sawTerminator = false;
  for (uint16_t i = 0; i < length; ++i) {
    const char c = static_cast<char>(data[i]);
    if (c == '\n' || c == '\r') {
      sawTerminator = true;
      if (bleRxLength) {
        bleRxBuffer[bleRxLength] = '\0';
        queueBleLine(bleRxBuffer);
        bleRxLength = 0;
      }
    } else if (bleRxLength < COMMAND_LINE_MAX) {
      bleRxBuffer[bleRxLength++] = c;
    } else {
      // v4.6: an over-long line disarms. loop() does that on the marker.
      bleRxLength = 0;
      const char marker[2] = {BLE_OVERFLOW_MARKER, '\0'};
      queueBleLine(marker);
    }
  }
  // v4.6 rule: a write with no terminator at all is one whole command.
  if (bleRxLength && !sawTerminator) {
    bleRxBuffer[bleRxLength] = '\0';
    queueBleLine(bleRxBuffer);
    bleRxLength = 0;
  }
}

void onBleConnect(uint16_t) {
  bleConnected = true;
  bleConnectEvent = true;
}

void onBleDisconnect(uint16_t, uint8_t) {
  bleConnected = false;
  bleDisconnectEvent = true;
  // Advertising restarts automatically through restartOnDisconnect(true).
}

// ---------------- Command intake (loop() only) ----------------

void handleBleIntake() {
  taskENTER_CRITICAL();
  const bool disconnected = bleDisconnectEvent;
  const bool connected = bleConnectEvent;
  const bool queueFull = bleQueueFull;
  bleDisconnectEvent = false;
  bleConnectEvent = false;
  bleQueueFull = false;
  taskEXIT_CRITICAL();

  if (disconnected) {
    // v4.6 disarms on every disconnect; a Pi-owned run carries on.
    if (controlOwner != SRC_SERIAL) neutralAndDisarm();
  }
  if (connected && bleConnected) {
    // The website may not have enabled notifications yet, so this line can be
    // lost; it also sends K on connect, which stops a Pi-owned run anyway.
    if (controlOwner == SRC_SERIAL) sendLine(SRC_BLE, "OWNER,SERIAL\n");
    else neutralAndDisarm();
  }

  char item[COMMAND_LINE_MAX + 1];
  while (xQueueReceive(bleLineQueue, item, 0) == pdPASS) {
    if (item[0] == BLE_OVERFLOW_MARKER && item[1] == '\0') neutralAndDisarm();
    else processCommand(String(item), SRC_BLE);
  }
  if (queueFull) processCommand(String("S"), SRC_BLE);
}

void handleSerialIntake() {
  // Serial is a stream: unlike BLE, a line only counts once it is terminated.
  for (int budget = SERIAL_READ_BUDGET; budget > 0 && Serial.available() > 0; --budget) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (!serialDiscarding && serialRxLength) {
        serialRxBuffer[serialRxLength] = '\0';
        processCommand(String(serialRxBuffer), SRC_SERIAL);
      }
      serialRxLength = 0;
      serialDiscarding = false;
    } else if (serialDiscarding) {
      // drop until the next terminator
    } else if (serialRxLength < COMMAND_LINE_MAX) {
      serialRxBuffer[serialRxLength++] = c;
    } else {
      serialRxLength = 0;
      serialDiscarding = true;
      if (controlOwner == SRC_SERIAL) neutralAndDisarm();
    }
  }
  // Host loss: the Pi closed the port or dropped DTR while driving.
  if (controlOwner == SRC_SERIAL && !Serial) neutralAndDisarm();
}

// ---------------- USB serial encoder report for the Raspberry Pi ----------------
// "MEAS,<left_rad_s>,<right_rad_s>,<left_rad>,<right_rad>\r\n" every 10 ms on USB
// serial, read by the Pi's teensy_interface_node and published as /wheel_states
// (sensor_msgs/JointState). Speed is measured over the real interval since the last
// sample (nominally 10 ms, longer if the loop was held up by BLE work), so a late
// sample never inflates the speed; if the loop falls behind, the schedule resyncs
// instead of bursting catch-up samples. Angle is total wheel rotation since power-up.
// Read-only: it only snapshots leftCount/rightCount and never touches motor, BLE or
// control state. Writes are skipped rather than blocking when USB is not attached
// or not being read. Lines the Pi sends are read by handleSerialIntake(); legacy
// REF lines do not parse as a command and are ignored.
constexpr uint32_t SERIAL_REPORT_INTERVAL_US = 10000;
constexpr float COUNTS_TO_RAD = 2.0f * PI / COUNTS_PER_OUTPUT_REV;
// Verify on the bench: rolling a wheel forward must give a positive value.
constexpr int8_t SERIAL_ENC_L_POLARITY = 1;
constexpr int8_t SERIAL_ENC_R_POLARITY = -1;
uint32_t lastSerialReportUs = 0;
uint32_t lastSerialSampleUs = 0;
int32_t serialPrevLeftCount = 0;
int32_t serialPrevRightCount = 0;

void updateSerialEncoderReport() {
  const uint32_t nowUs = micros();
  if (nowUs - lastSerialReportUs < SERIAL_REPORT_INTERVAL_US) return;
  if (nowUs - lastSerialReportUs >= 2 * SERIAL_REPORT_INTERVAL_US) {
    lastSerialReportUs = nowUs;  // fell behind: resync rather than burst
  } else {
    lastSerialReportUs += SERIAL_REPORT_INTERVAL_US;  // fixed schedule, no drift
  }
  const float elapsedS = (nowUs - lastSerialSampleUs) * 1.0e-6f;
  lastSerialSampleUs = nowUs;

  int32_t leftSnapshot;
  int32_t rightSnapshot;
  taskENTER_CRITICAL();
  leftSnapshot = leftCount;
  rightSnapshot = rightCount;
  taskEXIT_CRITICAL();
  const int32_t left = SERIAL_ENC_L_POLARITY * leftSnapshot;
  const int32_t right = SERIAL_ENC_R_POLARITY * rightSnapshot;

  const float leftRadS = (left - serialPrevLeftCount) * COUNTS_TO_RAD / elapsedS;
  const float rightRadS = (right - serialPrevRightCount) * COUNTS_TO_RAD / elapsedS;
  serialPrevLeftCount = left;
  serialPrevRightCount = right;

  char message[80];
  const int length = snprintf(
    message,
    sizeof(message),
    "MEAS,%.3f,%.3f,%.3f,%.3f\r\n",
    leftRadS,
    rightRadS,
    left * COUNTS_TO_RAD,
    right * COUNTS_TO_RAD
  );
  if (length <= 0 || length >= static_cast<int>(sizeof(message))) return;
  if (!Serial || Serial.availableForWrite() < length) return;  // drop, never block
  Serial.write(message, length);
}

void setup() {
  pinMode(BATTERY_PIN, INPUT);
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH); // XIAO nRF52840 Sense RGB LED is active-low.
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  leftPreviousState = (digitalRead(LEFT_ENCODER_A) << 1) | digitalRead(LEFT_ENCODER_B);
  rightPreviousState = (digitalRead(RIGHT_ENCODER_A) << 1) | digitalRead(RIGHT_ENCODER_B);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), updateRightEncoder, CHANGE);
 
  attachMotorPwm();
  neutralAndDisarm();

  // Created before the BLE callbacks can fire.
  bleLineQueue = xQueueCreate(BLE_QUEUE_DEPTH, COMMAND_LINE_MAX + 1);
 
  Bluefruit.begin(1, 0);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(onBleConnect);
  Bluefruit.Periph.setDisconnectCallback(onBleDisconnect);

  motorService.begin();

  txCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  txCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  txCharacteristic.setMaxLen(128);
  txCharacteristic.begin();

  rxCharacteristic.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  rxCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  rxCharacteristic.setMaxLen(128);
  rxCharacteristic.setWriteCallback(onRxWrite);
  rxCharacteristic.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(motorService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  // USB serial: encoder report and command input for the Pi; never waits for a host.
  Serial.begin(115200);
  lastSerialReportUs = micros();
  lastSerialSampleUs = lastSerialReportUs;

  const uint32_t now = millis();
  lastCommandMs = now;
  lastControlMs = now;
  lastSpeedMs = now;
  lastTelemetryMs = now;
  // Offset battery notifications from the 250 ms encoder telemetry slots.
  lastBatteryMs = now - (BATTERY_INTERVAL_MS - 125);
  lastLedToggleMs = now;
}

void updateBatteryMonitor(uint32_t now) {
  if (now - lastBatteryMs >= BATTERY_INTERVAL_MS) {
    lastBatteryMs = now;
    const uint16_t count = analogRead(BATTERY_PIN);
    batteryVoltage = batteryVoltageFromAdc(count);
    if (!lowBattery && batteryVoltage < LOW_BATTERY_TRIP_V) lowBattery = true;
    else if (lowBattery && batteryVoltage >= LOW_BATTERY_CLEAR_V) lowBattery = false;
    // One short notification per second; this path never changes armed state or PWM.
    char message[24];
    snprintf(message, sizeof(message), "BAT,%.2f,%d\n", batteryVoltage, lowBattery ? 1 : 0);
    sendLine(SRC_BOTH, message);
  }
  if (!lowBattery) {
    if (redLedOn) { digitalWrite(LED_RED, HIGH); redLedOn = false; }
    lastLedToggleMs = now;
  } else if (now - lastLedToggleMs >= LED_TOGGLE_INTERVAL_MS) {
    lastLedToggleMs = now;
    redLedOn = !redLedOn;
    digitalWrite(LED_RED, redLedOn ? LOW : HIGH);
  }
}
 
void loop() {
  // All commands, from either source, are handled here before the control code.
  handleBleIntake();
  handleSerialIntake();

  const uint32_t now = millis();
  updateSpeedEstimate(now);
 
  if (armed && now - lastCommandMs > SOFT_TIMEOUT_MS) {
    requestedLeft = 0.0f;
    requestedRight = 0.0f;
    requestedDirectPivot = false;
    waitingForNeutral = false;
    targetLeft = 0.0f;
    targetRight = 0.0f;
  }
  if (armed && now - lastCommandMs > HARD_TIMEOUT_MS) {
    neutralAndDisarm();
    sendLine(SRC_BOTH, "STATE,TIMEOUT_DISARMED\n");
  }
 
  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    const float dtSeconds = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;
    const float targetEnvelope = max(fabsf(targetLeft), fabsf(targetRight));
    motionEnvelope = stepEnvelope(motionEnvelope, targetEnvelope, dtSeconds);
    if (targetEnvelope > 0.0f) {
      // In-motion steering changes wheel ratio immediately, without
      // restarting the full drive ramp or returning to neutral.
      rampedLeft = targetLeft * motionEnvelope / targetEnvelope;
      rampedRight = targetRight * motionEnvelope / targetEnvelope;
    } else {
      // Preserve wheel direction while braking the shared speed envelope.
      const float previousEnvelope = max(fabsf(rampedLeft), fabsf(rampedRight));
      const float factor = previousEnvelope > 0.0f ? motionEnvelope / previousEnvelope : 0.0f;
      rampedLeft *= factor;
      rampedRight *= factor;
      if (motionEnvelope <= 0.0f) rampPivotMode = false;
    }
    updateSynchroniser(dtSeconds, now);
  }
 
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    int32_t leftSnapshot;
    int32_t rightSnapshot;
    taskENTER_CRITICAL();
    leftSnapshot = leftCount;
    rightSnapshot = rightCount;
    taskEXIT_CRITICAL();
    char message[128];
    snprintf(
      message,
      sizeof(message),
      "E,%ld,%ld,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d\n",
      static_cast<long>(leftSnapshot),
      static_cast<long>(rightSnapshot),
      leftRpm,
      rightRpm,
      appliedLeft,
      appliedRight,
      syncCorrection,
      armed ? 1 : 0,
      stallFault ? 1 : 0
    );
    notifyLine(message);
  }

  updateBatteryMonitor(now);
  updateSerialEncoderReport();

  delay(1);
}
