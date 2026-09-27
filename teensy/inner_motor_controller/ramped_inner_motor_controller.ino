
#include <Servo.h>
#include <Encoder.h>

// ---------------- Pins and polarity: COPY the values you settled on in motor_sysid.ino ----------------
constexpr int S1_PIN = 5;     // left  (purple)
constexpr int S2_PIN = 3;     // right (brown)
constexpr int ENC_L_A = 22;   // left  white
constexpr int ENC_L_B = 21;   // left  yellow
constexpr int ENC_R_A = 20;   // right white
constexpr int ENC_R_B = 19;   // right yellow
constexpr int8_t LEFT_POLARITY  = 1;
constexpr int8_t RIGHT_POLARITY = 1;
constexpr int8_t ENC_L_POLARITY = 1;
constexpr int8_t ENC_R_POLARITY = -1;
constexpr float COUNTS_PER_WHEEL_REV = 64.0f * 70.0f;

constexpr int NEUTRAL_PULSE_US = 1500;
constexpr int PULSE_SPAN_US = 500;

// ---------------- Gains: PASTE from analyse_motor_id.m (one run per wheel) ----------------
constexpr float KP_L       = 1.001587f;  // % per (rad/s)
constexpr float KI_L       = 23.663865f;  // % per rad
constexpr float DEADBAND_L = 1.270f;  // % feedforward
constexpr float KP_R       = 1.152035f;  // % per (rad/s)
constexpr float KI_R       = 27.408961f;  // % per rad
constexpr float DEADBAND_R = 1.625f;  // % feedforward

// ---------------- Loop and safety settings ----------------
constexpr uint32_t TS_US = 10000;        // 10 ms, same as identification
constexpr float TS = TS_US * 1.0e-6f;
constexpr float MAX_PERCENT = 25.0f;     // actuator limit, same as identification
constexpr float OVERSPEED_RADS = 18.0f;  // above what the motor can do -> something is wrong
constexpr uint32_t SAT_ABORT_MS = 1500;  // saturated this long -> wrong sign, stall or bad gains
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;  // no REF from the Pi this long -> stop the motors

// ---------------- Ramped target rate ----------------
// Smooth the command by limiting how much the wheel target can change each 10 ms cycle.
// This gives the motor a controlled rise time instead of a step response.
constexpr float MAX_RAMP_STEP_RAD_S = 2.5f;  // rad/s per 10 ms cycle; tune this for smoother motion

// ---------------- PI controller ----------------
struct WheelPI {
  float kp;
  float ki;
  float deadband; // range of motor commands where motor does not move
  float integral;
};

WheelPI piL{KP_L, KI_L, DEADBAND_L, 0.0f};
WheelPI piR{KP_R, KI_R, DEADBAND_R, 0.0f};

float updatePI(WheelPI &controller, float reference, float measured) {
  if (reference == 0.0f) {
    controller.integral = 0.0f;
    return 0.0f;
  }

  float error = reference - measured;
  float proportional = controller.kp * error;

  float deadbandCompensation = (reference > 0.0f) ? controller.deadband : -controller.deadband;

  float rawOutput = proportional + controller.integral + deadbandCompensation;
  float output = constrain(rawOutput, -MAX_PERCENT, MAX_PERCENT);

  bool saturatedHigh = rawOutput > MAX_PERCENT;
  bool saturatedLow  = rawOutput < -MAX_PERCENT;

  if (!saturatedHigh && !saturatedLow) {
    controller.integral += controller.ki * error * TS;
  } else if (saturatedHigh && error < 0.0f) {
    controller.integral += controller.ki * error * TS;
  } else if (saturatedLow && error > 0.0f) {
    controller.integral += controller.ki * error * TS;
  }

  return output;
}

float rampReference(float currentRef, float targetRef) {
  float delta = targetRef - currentRef;
  float maxStep = MAX_RAMP_STEP_RAD_S * TS / 0.01f; // convert to per-cycle equivalent

  if (delta > maxStep) {
    return currentRef + maxStep;
  }
  if (delta < -maxStep) {
    return currentRef - maxStep;
  }
  return targetRef;
}

// ---------------- Motor outputs ----------------
Servo leftMotorOutput;
Servo rightMotorOutput;

void writeMotorCommand(Servo &motorOutput, float percentCommand, int8_t polarity) {
  percentCommand = constrain(percentCommand, -MAX_PERCENT, MAX_PERCENT);
  float pulseWidthUs = NEUTRAL_PULSE_US + percentCommand * polarity * PULSE_SPAN_US / 100.0f;
  motorOutput.writeMicroseconds((int)lroundf(pulseWidthUs));
}

void neutral() {
  writeMotorCommand(leftMotorOutput, 0.0f, LEFT_POLARITY);
  writeMotorCommand(rightMotorOutput, 0.0f, RIGHT_POLARITY);
}

// ---------------- Encoders ----------------
Encoder encL(ENC_L_A, ENC_L_B);
Encoder encR(ENC_R_A, ENC_R_B);

int32_t prevCountL = 0;
int32_t prevCountR = 0;

constexpr float COUNTS_TO_RAD_PER_SEC = 2.0f * PI / (COUNTS_PER_WHEEL_REV * TS);

float measureWheelSpeed(Encoder &encoder, int32_t &prevCount, int8_t polarity) {
  int32_t count = polarity * encoder.read();
  float speed = (count - prevCount) * COUNTS_TO_RAD_PER_SEC;
  prevCount = count;
  return speed;
}

// ---------------- References: set by REF,<left>,<right> lines from the Pi ----------------
float referenceLeft = 0.0f;
float referenceRight = 0.0f;
float rampedLeft = 0.0f;
float rampedRight = 0.0f;
uint32_t lastCommandMillis = 0;

// ---------------- Saturation tracking, for the SAT_ABORT_MS safety check ----------------
uint32_t satDurationMsL = 0;
uint32_t satDurationMsR = 0;

// ---------------- Per-cycle update: run once every TS seconds ----------------
void tick() {
  float measuredLeft  = measureWheelSpeed(encL, prevCountL, ENC_L_POLARITY);
  float measuredRight = measureWheelSpeed(encR, prevCountR, ENC_R_POLARITY);

  if (fabs(measuredLeft) > OVERSPEED_RADS || fabs(measuredRight) > OVERSPEED_RADS) {
    neutral();
    referenceLeft = 0.0f;
    referenceRight = 0.0f;
    rampedLeft = 0.0f;
    rampedRight = 0.0f;
    Serial.println("#ABORT,OVERSPEED");
    return;
  }

  rampedLeft = rampReference(rampedLeft, referenceLeft);
  rampedRight = rampReference(rampedRight, referenceRight);

  float commandLeft  = updatePI(piL, rampedLeft, measuredLeft);
  float commandRight = updatePI(piR, rampedRight, measuredRight);

  satDurationMsL = (fabs(commandLeft)  >= MAX_PERCENT) ? satDurationMsL + TS_US / 1000 : 0;
  satDurationMsR = (fabs(commandRight) >= MAX_PERCENT) ? satDurationMsR + TS_US / 1000 : 0;
  if (satDurationMsL > SAT_ABORT_MS || satDurationMsR > SAT_ABORT_MS) {
    neutral();
    referenceLeft = 0.0f;
    referenceRight = 0.0f;
    rampedLeft = 0.0f;
    rampedRight = 0.0f;
    Serial.println("#ABORT,SATURATED_TOO_LONG");
    return;
  }

  writeMotorCommand(leftMotorOutput, commandLeft, LEFT_POLARITY);
  writeMotorCommand(rightMotorOutput, commandRight, RIGHT_POLARITY);

  Serial.print("MEAS,");
  Serial.print(measuredLeft, 3);
  Serial.print(',');
  Serial.println(measuredRight, 3);
}

// ---------------- Serial input: "REF,<left_rad_s>,<right_rad_s>\n" ----------------
void handleSerialInput() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.startsWith("REF,")) return;

  int firstComma  = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);
  if (secondComma < 0) return;

  referenceLeft  = line.substring(firstComma + 1, secondComma).toFloat();
  referenceRight = line.substring(secondComma + 1).toFloat();
  lastCommandMillis = millis();
}

elapsedMicros sinceTick;

void setup() {
  leftMotorOutput.attach(S1_PIN, NEUTRAL_PULSE_US - PULSE_SPAN_US, NEUTRAL_PULSE_US + PULSE_SPAN_US);
  rightMotorOutput.attach(S2_PIN, NEUTRAL_PULSE_US - PULSE_SPAN_US, NEUTRAL_PULSE_US + PULSE_SPAN_US);
  neutral();

  Serial.begin(115200);
  Serial.setTimeout(1);
  delay(500);
  neutral();
  lastCommandMillis = millis();
  rampedLeft = 0.0f;
  rampedRight = 0.0f;
  Serial.println("#READY,RAMPED_INNER_MOTOR_CONTROLLER");
}

void loop() {
  handleSerialInput();

  if (millis() - lastCommandMillis > COMMAND_TIMEOUT_MS) {
    referenceLeft = 0.0f;
    referenceRight = 0.0f;
  }

  if (sinceTick >= TS_US) {
    sinceTick -= TS_US;
    tick();
  }
}
