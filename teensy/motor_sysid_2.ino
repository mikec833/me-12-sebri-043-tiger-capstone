// motor_sysid.ino
// Open-loop step-test logger for identifying a first-order motor model
//     Thetadot(s) / U(s) = K / (tau*s + 1)     (L08 slide 8, "Motor model" block)
//   U        = Sabertooth command in %  (proportional to motor voltage V = U/100 * V_batt)
//   Thetadot = WHEEL angular velocity in rad/s (theta_l dot, theta_r dot in the kinematic model)
//   Sign convention: positive = wheel rolling the robot FORWARD (+x_B), on both sides.
//
// Board:  Teensy 3.2
// Driver: Sabertooth 2x12, independent R/C mode, linear, loss-of-signal timeout on
//
// >>> WHEELS RAISED OFF THE GROUND FOR THIS TEST <<<
//
// Usage (USB serial, 115200):
//   send 'l' = step left wheel only, 'r' = right only, 'b' = both
//   any other key while running, or closing the serial port, aborts -> neutral
//
// Output: header "t_ms,cmdL,cmdR,countL,countR,thdotL,thdotR" then one CSV row per sample.
//         Status lines start with '#'. Run ends with "#END".
//
// Row k holds the command that was applied during the sample interval ending at t_ms,
// and the average wheel angular velocity (rad/s) measured over that same interval.

#include <Servo.h>
#include <Encoder.h>   // PJRC Encoder library, ships with Teensyduino (4x quadrature decoding)

// ---------------- Motor outputs (copied from bench sketch) ----------------
constexpr int S1_PIN = 5;   // left  -> Sabertooth S1 (purple)
constexpr int S2_PIN = 3;   // right -> Sabertooth S2 (brown)
constexpr int NEUTRAL_PULSE_US = 1500;
constexpr int PULSE_SPAN_US = 500;
constexpr int8_t LEFT_POLARITY = 1;
constexpr int8_t RIGHT_POLARITY = 1;

// ---------------- Encoders: SET THESE ----------------
// Any digital pin on a Teensy 3.2 is interrupt capable and 5 V tolerant.
// WARNING: pin 19 is also I2C SCL0 (and 18 is SDA0). If the IMU goes on the
// default I2C bus it will clash with ENC_R_B - move that encoder channel or use Wire1.
constexpr int ENC_L_A = 22;   // encoder 1 (left)  white
constexpr int ENC_L_B = 21;   // encoder 1 (left)  yellow
constexpr int ENC_R_A = 20;   // encoder 2 (right) white
constexpr int ENC_R_B = 19;   // encoder 2 (right) yellow

// The motors are mirrored on the chassis, so one side usually needs flipping.
// Set LEFT/RIGHT_POLARITY so a positive command rolls that wheel FORWARD, then set
// ENC_x_POLARITY so forward rotation gives a POSITIVE thdot. Check with a 10% command.
constexpr int8_t ENC_L_POLARITY = 1;
constexpr int8_t ENC_R_POLARITY = -1;

// Counts per WHEEL revolution as seen by the Encoder library (4x decoding):
//   = (encoder lines per MOTOR rev) * 4 * gear_ratio
// If the datasheet CPR is already quadrature counts, drop the * 4.
// Must be counts per WHEEL rev (i.e. include the gearbox) so thdot is theta_l/theta_r dot.
// Placeholder value - thdot columns are wrong until this is set, but the raw
// count columns are always valid so MATLAB recomputes thdot anyway.
constexpr float COUNTS_PER_WHEEL_REV = 64.0f * 70.0f;  // 4480 counts/rev
// ---------------- Test settings ----------------
constexpr uint32_t TS_US = 10000;   // 10 ms sample period (100 Hz)
constexpr int MAX_PERCENT = 25;     // hard clamp on any command; lower for a first run

struct Step { int8_t pct; uint16_t ms; };

// Each hold must be long enough for speed to settle (> ~5 tau). If the logged
// steps still ramping at the end, lengthen the 3000 ms holds.
const Step SCHEDULE[] = {
  {  0, 1000},
  // small staircase: finds the deadband / breakaway command
  {  3, 1500}, {  6, 1500}, {  9, 1500}, { 12, 1500}, {  0, 2000},
  // steps up from rest at increasing amplitude: checks linearity of K and tau
  { 20, 3000}, {  0, 3000},
  { 40, 3000}, {  0, 3000},
  { 60, 3000}, {  0, 3000},
  // steps between non-zero levels: small-signal behaviour around a cruise speed
  { 30, 3000}, { 50, 3000}, { 30, 3000}, {  0, 3000},
  // reverse direction
  { -3, 1500}, { -6, 1500}, { -9, 1500}, {-12, 1500}, {  0, 2000},
  {-20, 3000}, {  0, 3000},
  {-40, 3000}, {  0, 3000},
  {-60, 3000}, {  0, 3000},
};
constexpr size_t NUM_STEPS = sizeof(SCHEDULE) / sizeof(SCHEDULE[0]);

// ---------------- State ----------------
Servo leftOut;
Servo rightOut;
Encoder encL(ENC_L_A, ENC_L_B);
Encoder encR(ENC_R_A, ENC_R_B);

bool running = false;
char wheelSel = 'b';
size_t stepIdx = 0;
uint32_t stepMs = 0;
uint32_t sampleN = 0;
int32_t prevL = 0;
int32_t prevR = 0;
int cmdL = 0;
int cmdR = 0;
elapsedMicros sinceTick;

int pctToPulse(int pct, int polarity) {
  pct = constrain(pct, -MAX_PERCENT, MAX_PERCENT);
  return NEUTRAL_PULSE_US + (pct * polarity * PULSE_SPAN_US) / 100;
}

void setOutputs(int left, int right) {
  cmdL = constrain(left, -MAX_PERCENT, MAX_PERCENT);
  cmdR = constrain(right, -MAX_PERCENT, MAX_PERCENT);
  leftOut.writeMicroseconds(pctToPulse(cmdL, LEFT_POLARITY));
  rightOut.writeMicroseconds(pctToPulse(cmdR, RIGHT_POLARITY));
}

void applyStep() {
  const int pct = SCHEDULE[stepIdx].pct;
  setOutputs(wheelSel != 'r' ? pct : 0,
             wheelSel != 'l' ? pct : 0);
}

void stopRun(const char *msg) {
  setOutputs(0, 0);
  running = false;
  Serial.println(msg);
}

void startRun(char sel) {
  wheelSel = sel;
  encL.write(0);
  encR.write(0);
  prevL = 0;
  prevR = 0;
  stepIdx = 0;
  stepMs = 0;
  sampleN = 0;
  Serial.println("t_ms,cmdL,cmdR,countL,countR,thdotL,thdotR");
  applyStep();
  sinceTick = 0;
  running = true;
}

void tick() {
  const int32_t cL = ENC_L_POLARITY * encL.read();
  const int32_t cR = ENC_R_POLARITY * encR.read();

  // counts per sample -> wheel angular velocity in rad/s
  const float countsToRadS = 2.0f * PI * 1.0e6f / (COUNTS_PER_WHEEL_REV * (float)TS_US);
  const float thdotL = (float)(cL - prevL) * countsToRadS;
  const float thdotR = (float)(cR - prevR) * countsToRadS;
  prevL = cL;
  prevR = cR;
  ++sampleN;

  // Log the command that was active during the interval just measured.
  Serial.print(sampleN * (TS_US / 1000)); Serial.print(',');
  Serial.print(cmdL);    Serial.print(',');
  Serial.print(cmdR);    Serial.print(',');
  Serial.print(cL);      Serial.print(',');
  Serial.print(cR);      Serial.print(',');
  Serial.print(thdotL, 3); Serial.print(',');
  Serial.println(thdotR, 3);

  // Advance the schedule on sample boundaries so step times are exact.
  stepMs += TS_US / 1000;
  if (stepMs >= SCHEDULE[stepIdx].ms) {
    stepMs = 0;
    if (++stepIdx >= NUM_STEPS) {
      stopRun("#END");
      return;
    }
    applyStep();
  }
}

void setup() {
  leftOut.attach(S1_PIN, NEUTRAL_PULSE_US - PULSE_SPAN_US, NEUTRAL_PULSE_US + PULSE_SPAN_US);
  rightOut.attach(S2_PIN, NEUTRAL_PULSE_US - PULSE_SPAN_US, NEUTRAL_PULSE_US + PULSE_SPAN_US);
  setOutputs(0, 0);

  Serial.begin(115200);
  delay(500);
  setOutputs(0, 0);
  Serial.println("#READY,MOTOR_SYSID,send l / r / b to start");
}

void loop() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;   // serial monitor line endings
    if (running) {
      stopRun("#ABORT,USER");
    } else if (c == 'l' || c == 'r' || c == 'b') {
      startRun(c);
    } else {
      Serial.println("#ERR,send l, r or b");
    }
  }

  // Host closed the port -> stop.
  if (running && !Serial.dtr()) {
    setOutputs(0, 0);
    running = false;
  }

  if (running && sinceTick >= TS_US) {
    sinceTick -= TS_US;   // subtract rather than zero, so timing does not drift
    tick();
  }
}
