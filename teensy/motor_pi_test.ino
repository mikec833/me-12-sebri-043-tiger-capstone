// motor_pi_test.ino
// Closed-loop wheel speed test: the inner-loop "Motor control" block (L08 slide 13).
//   reference thdot_ref (rad/s) -> PI -> Sabertooth command (%) -> motor -> encoder -> thdot
//
// Board:  Teensy 3.2        Driver: Sabertooth 2x12, R/C mode
// >>> WHEELS RAISED for the first test <<<
//
// Usage (USB serial, 115200), same as motor_sysid:
//   'l' = left only, 'r' = right only, 'b' = both.  Any other key or closing the port aborts.
// Output CSV: t_ms,refL,refR,cmdL,cmdR,countL,countR,thdotL,thdotR
//   Row k: thdot = average speed over the interval ending at t_ms,
//          cmd   = PI output computed from it (applied for the next interval).

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
constexpr int8_t RIGHT_POLARITY = -1;
constexpr int8_t ENC_L_POLARITY = 1;
constexpr int8_t ENC_R_POLARITY = 1;
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
constexpr bool ZERO_REF_NEUTRAL = true;  // ref == 0 -> output exactly neutral and clear integrator

// ---------------- Reference schedule (wheel speed, rad/s) ----------------
// Identified max is ~13 rad/s at 25 %, so stay at or below 10 to leave headroom for the PI.
struct Step { float ref; uint16_t ms; };
const Step SCHEDULE[] = {
  {  0, 1000},
  {  2, 2000}, {  4, 2000}, {  6, 2000}, {  8, 2000}, { 10, 2000}, {  0, 2000},
  { 10, 2000}, {  0, 2000}, { 15, 2000}, {  0, 2000},
  { 20, 2000}, {  0, 2000}, { 25, 2000}, {  0, 2000},
  { 10, 2000}, { 20, 2000}, { 10, 2000}, {  0, 2000},
  { -2, 2000}, { -4, 2000}, { -6, 2000}, { -8, 2000}, {-10, 2000}, {  0, 2000},
  {-10, 2000}, {  0, 2000}, {-15, 2000}, {  0, 2000},
  {-20, 2000}, {  0, 2000}, {-25, 2000}, {  0, 2000},
  {-10, 2000}, {-20, 2000}, {-10, 2000}, {  0, 2000},
};
constexpr size_t NUM_STEPS = sizeof(SCHEDULE) / sizeof(SCHEDULE[0]);

// ---------------- PI controller ----------------
struct WheelPI {
  float kp, ki, dff;
  float integ;
  uint32_t satMs;
};

WheelPI piL{KP_L, KI_L, DEADBAND_L, 0.0f, 0};
WheelPI piR{KP_R, KI_R, DEADBAND_R, 0.0f, 0};

float updatePI(WheelPI &c, float ref, float meas) {
  if (ZERO_REF_NEUTRAL && ref == 0.0f) {
    c.integ = 0.0f;
    c.satMs = 0;
    return 0.0f;
  }

  const float e  = ref - meas;
  const float ff = (ref > 0.0f) ? c.dff : -c.dff;          // deadband feedforward
  const float uUnsat = c.kp * e + c.integ + ff;
  const float u = constrain(uUnsat, -MAX_PERCENT, MAX_PERCENT);

  // Anti-windup: only integrate if not saturated, or if the error would pull out of saturation
  if (u == uUnsat || ((e > 0.0f) != (uUnsat > 0.0f))) {
    c.integ += c.ki * e * TS;
  }

  c.satMs = (u != uUnsat) ? c.satMs + TS_US / 1000 : 0;
  return u;
}

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
elapsedMicros sinceTick;

void writePercent(Servo &out, float pct, int8_t polarity) {
  pct = constrain(pct, -MAX_PERCENT, MAX_PERCENT);
  const float us = NEUTRAL_PULSE_US + pct * polarity * PULSE_SPAN_US / 100.0f;
  out.writeMicroseconds((int)lroundf(us));
}

void neutral() {
  writePercent(leftOut, 0.0f, LEFT_POLARITY);
  writePercent(rightOut, 0.0f, RIGHT_POLARITY);
}

void stopRun(const char *msg) {
  neutral();
  running = false;
  Serial.println(msg);
}

void startRun(char sel) {
  const bool needL = (sel != 'r');
  const bool needR = (sel != 'l');
  if ((needL && KP_L == 0.0f) || (needR && KP_R == 0.0f)) {
    Serial.println("#ERR,paste the gains for that wheel first");
    return;
  }
  wheelSel = sel;
  encL.write(0);
  encR.write(0);
  prevL = prevR = 0;
  piL.integ = piR.integ = 0.0f;
  piL.satMs = piR.satMs = 0;
  stepIdx = 0;
  stepMs = 0;
  sampleN = 0;
  Serial.println("t_ms,refL,refR,cmdL,cmdR,countL,countR,thdotL,thdotR");
  sinceTick = 0;
  running = true;
}

void tick() {
  // 1. Measure wheel speed over the last interval
  const int32_t cL = ENC_L_POLARITY * encL.read();
  const int32_t cR = ENC_R_POLARITY * encR.read();
  const float countsToRadS = 2.0f * PI / (COUNTS_PER_WHEEL_REV * TS);
  const float thL = (cL - prevL) * countsToRadS;
  const float thR = (cR - prevR) * countsToRadS;
  prevL = cL;
  prevR = cR;

  // 2. References for this sample
  const float r = SCHEDULE[stepIdx].ref;
  const float refL = (wheelSel != 'r') ? r : 0.0f;
  const float refR = (wheelSel != 'l') ? r : 0.0f;

  // 3. PI and output
  const float uL = updatePI(piL, refL, thL);
  const float uR = updatePI(piR, refR, thR);
  writePercent(leftOut, uL, LEFT_POLARITY);
  writePercent(rightOut, uR, RIGHT_POLARITY);

  // 4. Log
  ++sampleN;
  Serial.print(sampleN * (TS_US / 1000)); Serial.print(',');
  Serial.print(refL, 2); Serial.print(',');
  Serial.print(refR, 2); Serial.print(',');
  Serial.print(uL, 2);   Serial.print(',');
  Serial.print(uR, 2);   Serial.print(',');
  Serial.print(cL);      Serial.print(',');
  Serial.print(cR);      Serial.print(',');
  Serial.print(thL, 3);  Serial.print(',');
  Serial.println(thR, 3);

  // 5. Safety checks
  if (fabs(thL) > OVERSPEED_RADS || fabs(thR) > OVERSPEED_RADS) {
    stopRun("#ABORT,OVERSPEED");
    return;
  }
  if (piL.satMs > SAT_ABORT_MS || piR.satMs > SAT_ABORT_MS) {
    stopRun("#ABORT,SATURATED_TOO_LONG (check polarity / gains)");
    return;
  }

  // 6. Advance the schedule
  stepMs += TS_US / 1000;
  if (stepMs >= SCHEDULE[stepIdx].ms) {
    stepMs = 0;
    if (++stepIdx >= NUM_STEPS) {
      stopRun("#END");
    }
  }
}

void setup() {
  leftOut.attach(S1_PIN, NEUTRAL_PULSE_US - PULSE_SPAN_US, NEUTRAL_PULSE_US + PULSE_SPAN_US);
  rightOut.attach(S2_PIN, NEUTRAL_PULSE_US - PULSE_SPAN_US, NEUTRAL_PULSE_US + PULSE_SPAN_US);
  neutral();
  Serial.begin(115200);
  delay(500);
  neutral();
  Serial.println("#READY,MOTOR_PI_TEST,send l / r / b to start");
}

void loop() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') continue;
    if (running) {
      stopRun("#ABORT,USER");
    } else if (c == 'l' || c == 'r' || c == 'b') {
      startRun(c);
    } else {
      Serial.println("#ERR,send l, r or b");
    }
  }

  if (running && !Serial.dtr()) {
    neutral();
    running = false;
  }

  if (running && sinceTick >= TS_US) {
    sinceTick -= TS_US;
    tick();
  }
}
