/*
 * teensy_bridge.ino -- two-motor + two-encoder bridge with closed-loop
 * wheel-speed control (inner loop lives here on the Teensy -- see project
 * notes on why: the encoders are read right here with no serial round trip,
 * so this is the lowest-latency place to close the speed loop. The Pi/GUI
 * just sends a target; this file is responsible for actually hitting it).
 *
 * Motors: Sabertooth 2x12 S1 (motor 1) / S2 (motor 2) inputs (R/C-style
 *         pulse, via Servo library).
 * Serial in:  "M,<target1>,<target2>" (-1..1 each -- a fraction of
 *             MAX_TICKS_PER_SEC, not a direct pulse width anymore), or "S"
 *             to stop both.
 * Serial out: "E,<count1>,<rate1>,<count2>,<rate2>,<millis>,<a1>,<b1>,<a2>,<b2>"
 *             every 100ms (a1/b1/a2/b2 are the raw encoder pin states --
 *             handy for checking wiring: they should visibly toggle as
 *             each wheel turns even before you trust the tick count).
 * Motors auto-stop if no command arrives within 500ms -- this bypasses the
 * PID entirely and forces an immediate neutral pulse rather than waiting
 * for closed-loop convergence to zero; see the timeout handling in loop().
 *
 * LIBRARY NEEDED: "PID" by Brett Beauregard (PID_v1), via Library Manager.
 *
 * TUNING: KP/KI/KD and MAX_TICKS_PER_SEC below are starting points, not
 * real values -- "good rise time, minimal overshoot" is a property of
 * whatever gains you actually put in, the library just executes them
 * mechanically. Use the step-test logging (send a step in M,<target>,0 and
 * watch the E, output) to actually tune these; don't trust the defaults.
 */
#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

constexpr int ENC_1_A_PIN = 21;  // motor 1 encoder
constexpr int ENC_1_B_PIN = 22;
constexpr int ENC_2_A_PIN = 19;  // motor 2 encoder
constexpr int ENC_2_B_PIN = 20;

constexpr int S1_PIN = 5;  // motor 1 Sabertooth signal
constexpr int S2_PIN = 3;  // motor 2 Sabertooth signal

constexpr int PULSE_NEUTRAL_US = 1500;
constexpr int PULSE_MAX_US     = 2000;
constexpr int PULSE_SPAN_US    = PULSE_MAX_US - PULSE_NEUTRAL_US;  // +/- range the PID output is clamped to

constexpr unsigned long CMD_TIMEOUT_MS    = 500;
constexpr unsigned long ENCODER_STREAM_MS = 100;  // also the PID's fixed sample time -- see setup()

// TODO: measure this per motor -- command close to full speed open-loop
// (temporarily bypass the PID, or just watch the transient right after a
// step before it's tuned) and read the settled ticks/sec off the E, output.
// This is what turns a -1..1 command into an actual target speed.
constexpr double MAX_TICKS_PER_SEC = 2000.0;

// TODO: tune via the step-test/Bode approach, not left as a guess.
constexpr double KP = 0.5, KI = 2.0, KD = 0.0;

Encoder encoder1(ENC_1_A_PIN, ENC_1_B_PIN);
Encoder encoder2(ENC_2_A_PIN, ENC_2_B_PIN);
Servo motor1;
Servo motor2;

float cmdSpeed1 = 0.0f, cmdSpeed2 = 0.0f;  // raw -1..1 command from serial
unsigned long lastCmdMs = 0;
long lastCount1 = 0, lastCount2 = 0;
unsigned long lastStreamMs = 0;

// PID_v1 reads/writes these directly by pointer.
double targetSpeed1 = 0, measuredSpeed1 = 0, pidOutput1 = 0;
double targetSpeed2 = 0, measuredSpeed2 = 0, pidOutput2 = 0;
PID speedPID1(&measuredSpeed1, &pidOutput1, &targetSpeed1, KP, KI, KD, DIRECT);
PID speedPID2(&measuredSpeed2, &pidOutput2, &targetSpeed2, KP, KI, KD, DIRECT);

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.startsWith("M,")) {
    int comma = line.indexOf(',', 2);
    if (comma > 0) {
      cmdSpeed1 = line.substring(2, comma).toFloat();
      cmdSpeed2 = line.substring(comma + 1).toFloat();
      lastCmdMs = millis();
    }
  } else if (line == "S") {
    cmdSpeed1 = 0.0f;
    cmdSpeed2 = 0.0f;
    lastCmdMs = millis();
  }
}

// Forces an immediate open-loop neutral pulse and clears each PID's
// accumulated integral term (the SetMode(MANUAL)->SetMode(AUTOMATIC)
// round-trip is PID_v1's documented way to reset internal state without
// an output bump). Used only on command timeout -- see loop().
void emergencyNeutral() {
  motor1.writeMicroseconds(PULSE_NEUTRAL_US);
  motor2.writeMicroseconds(PULSE_NEUTRAL_US);
  speedPID1.SetMode(MANUAL); speedPID1.SetMode(AUTOMATIC);
  speedPID2.SetMode(MANUAL); speedPID2.SetMode(AUTOMATIC);
}

void setup() {
  Serial.begin(115200);
  motor1.attach(S1_PIN);
  motor2.attach(S2_PIN);
  pinMode(ENC_1_A_PIN, INPUT_PULLUP);
  pinMode(ENC_1_B_PIN, INPUT_PULLUP);
  pinMode(ENC_2_A_PIN, INPUT_PULLUP);
  pinMode(ENC_2_B_PIN, INPUT_PULLUP);

  speedPID1.SetOutputLimits(-PULSE_SPAN_US, PULSE_SPAN_US);
  speedPID2.SetOutputLimits(-PULSE_SPAN_US, PULSE_SPAN_US);
  speedPID1.SetSampleTime(ENCODER_STREAM_MS);  // matches how often measuredSpeed actually gets new data
  speedPID2.SetSampleTime(ENCODER_STREAM_MS);
  speedPID1.SetMode(AUTOMATIC);
  speedPID2.SetMode(AUTOMATIC);

  emergencyNeutral();
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  bool timedOut = (millis() - lastCmdMs > CMD_TIMEOUT_MS);

  if (now - lastStreamMs >= ENCODER_STREAM_MS) {
    long count1 = encoder1.read();
    long count2 = encoder2.read();
    float dt_s = (now - lastStreamMs) / 1000.0f;
    float ticksPerSec1 = (count1 - lastCount1) / dt_s;
    float ticksPerSec2 = (count2 - lastCount2) / dt_s;
    lastCount1 = count1;
    lastCount2 = count2;
    lastStreamMs = now;

    Serial.print("E,");
    Serial.print(count1); Serial.print(',');
    Serial.print(ticksPerSec1, 2); Serial.print(',');
    Serial.print(count2); Serial.print(',');
    Serial.print(ticksPerSec2, 2); Serial.print(',');
    Serial.print(now); Serial.print(',');
    Serial.print(digitalRead(ENC_1_A_PIN)); Serial.print(',');
    Serial.print(digitalRead(ENC_1_B_PIN)); Serial.print(',');
    Serial.print(digitalRead(ENC_2_A_PIN)); Serial.print(',');
    Serial.println(digitalRead(ENC_2_B_PIN));

    // Closed loop: only while we have a live command. On timeout,
    // emergencyNeutral() (below) takes over instead -- don't let the PID
    // spend several cycles "converging" to zero when the safety watchdog
    // wants an immediate stop.
    if (!timedOut) {
      measuredSpeed1 = ticksPerSec1;
      measuredSpeed2 = ticksPerSec2;
      targetSpeed1 = cmdSpeed1 * MAX_TICKS_PER_SEC;
      targetSpeed2 = cmdSpeed2 * MAX_TICKS_PER_SEC;
      speedPID1.Compute();
      speedPID2.Compute();
      motor1.writeMicroseconds(PULSE_NEUTRAL_US + (int)pidOutput1);
      motor2.writeMicroseconds(PULSE_NEUTRAL_US + (int)pidOutput2);
    }
  }

  if (timedOut) {
    cmdSpeed1 = 0.0f;
    cmdSpeed2 = 0.0f;
    emergencyNeutral();
  }
}
