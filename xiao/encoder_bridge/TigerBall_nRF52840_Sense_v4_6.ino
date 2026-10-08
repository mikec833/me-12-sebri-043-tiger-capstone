#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <Servo.h>
 
// TigerBall 500 mm bench controller: XIAO nRF52840 -> Sabertooth 2x12.
// Arduino board package: Seeed nRF52 Boards (non-mbed).
// Use the Bluefruit and Servo libraries bundled with that board package.
// Sabertooth must be physically configured for independent R/C input mode.
 
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
String receiveBuffer;
 
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
 
void neutralAndDisarm() {
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
  // survive a disconnect, fault, STOP, or tab-change disarm.
  configuredMaximumPercent = 10.0f;
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
      notifyLine("STATE,WAIT_NEUTRAL\n");
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
      notifyLine("STATE,NEUTRAL_READY\n");
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
      notifyLine("FAULT,ENCODER_STALL\n");
    }
  } else {
    stallStartedMs = 0;
  }
}
 
void processCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  if (command == "V") {
    notifyLine("VERSION,4.6\n");
    return;
  }
 
  if (command == "A") {
    if (!stallFault) {
      const float armedMaximum = configuredMaximumPercent;
      neutralAndDisarm();
      configuredMaximumPercent = armedMaximum;
      armed = true;
      lastCommandMs = millis();
      notifyLine("STATE,ARMED\n");
    }
    return;
  }
  if (command == "K") {
    neutralAndDisarm();
    notifyLine("STATE,DISARMED\n");
    return;
  }
  if (command == "R") {
    neutralAndDisarm();
    stallFault = false;
    notifyLine("STATE,FAULT_RESET\n");
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
      lastCommandMs = millis();
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
    accelerationMs = static_cast<uint32_t>(first);
    brakingMs = static_cast<uint32_t>(second);
    configuredMaximumPercent = third;
    return;
  }
  if (*end != '\0') return;
  if (type == 'M' || type == 'I' || type == 'B' || type == 'P') {
    if (fabsf(first) > OUTPUT_LIMIT_PERCENT || fabsf(second) > OUTPUT_LIMIT_PERCENT) return;
    // P is a short-ramp opposite-wheel pivot; legacy I cannot bypass the gate.
    if (type == 'P' && (first * second >= 0.0f || fabsf(first + second) > 1.0f)) return;
    setTargets(first, second, type == 'P');
  }
}
 
void onRxWrite(uint16_t, BLECharacteristic *, uint8_t *data, uint16_t length) {
  String incoming;
  incoming.reserve(length);
  for (uint16_t i = 0; i < length; ++i) {
    incoming += static_cast<char>(data[i]);
  }
  for (size_t i = 0; i < incoming.length(); ++i) {
    const char c = incoming[i];
    if (c == '\n' || c == '\r') {
      if (receiveBuffer.length()) {
        processCommand(receiveBuffer);
        receiveBuffer = "";
      }
    } else if (receiveBuffer.length() < 80) {
      receiveBuffer += c;
    } else {
      receiveBuffer = "";
      neutralAndDisarm();
    }
  }
  if (receiveBuffer.length() && incoming.indexOf('\n') < 0 && incoming.indexOf('\r') < 0) {
    processCommand(receiveBuffer);
    receiveBuffer = "";
  }
}

void onBleConnect(uint16_t) {
  bleConnected = true;
  neutralAndDisarm();
}

void onBleDisconnect(uint16_t, uint8_t) {
  bleConnected = false;
  neutralAndDisarm();
  // Advertising restarts automatically through restartOnDisconnect(true).
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
// or not being read, and anything the Pi sends (REF lines) is discarded.
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
  while (Serial.available() > 0) Serial.read();

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

  // USB serial encoder report for the Pi; never waits for a host.
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
    notifyLine(message);
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
    notifyLine("STATE,TIMEOUT_DISARMED\n");
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
