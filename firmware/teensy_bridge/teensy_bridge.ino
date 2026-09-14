/*
 * teensy_bridge.ino
 * ---------------------------------------------------------------------------
 * Teensy-side firmware for today's Pi + Teensy integration setup:
 *     Pi motor cmd    -----> Sabertooth 2x12 S1 input (R/C mode)
 *     Encoder ticks   -----> streamed to the Pi over USB-serial
 *
 * The Pi is the brain: it runs the UWB pose script (separately), reads the
 * BNO085 IMU directly over its own UART (the IMU is wired straight to the
 * Pi, NOT to this Teensy -- there's nothing IMU-related in this file), and
 * runs a simple teleop loop, talking to this Teensy over the micro-USB ->
 * USB-A cable. This sketch's only jobs are: drive the motor when told to,
 * report the encoder, and NOT drive the motor if it stops hearing from the
 * Pi.
 *
 * SERIAL PROTOCOL (both directions are plain text, newline-terminated):
 *   Teensy -> Pi:  "E,<ticks>,<ticks_per_s>,<millis>"      sent every ENCODER_STREAM_MS
 *   Pi -> Teensy:  "M,<speed>"                             motor 1 command, speed in [-1, 1]
 *                  "S"                                     stop
 *
 * MOTOR NOTE: this drives a Sabertooth 2x12 over its S1 input, which is an
 * R/C-style input -- NOT a duty-cycle PWM + direction pin like a plain
 * H-bridge (L298N/TB6612/etc). It wants a repeating servo/ESC-style pulse:
 * ~1000us = full reverse, ~1500us = stop, ~2000us = full forward. We
 * generate that with the standard Arduino Servo library (Servo.attach() +
 * writeMicroseconds()) -- it's not actually driving a servo, but the signal
 * shape is identical and the library already handles the timing correctly.
 * "Stop" means continuously sending the 1500us neutral pulse, not silence --
 * the Sabertooth expects a live signal at all times in R/C mode. Only S1 is
 * used right now (one motor); S2 is free for a second motor later, at which
 * point this needs a second Servo object + a second setMotorN() and, if you
 * want body-frame v/w control again, differential-drive mixing back on top.
 * Double-check the Sabertooth's DIP switches are actually set for
 * (independent, non-mixed) R/C mode -- see its manual.
 *
 * ENCODER NOTE: quadrature encoder (2 channels, A/B) on MOTOR1_ENC_A_PIN/
 * MOTOR1_ENC_B_PIN. This uses the "Encoder" library (PJRC/Paul Stoffregen)
 * rather than a hand-written interrupt -- worth understanding WHY: polling
 * an encoder in loop() can miss transitions between iterations, especially
 * as the motor speeds up, so *something* needs to catch every edge as it
 * happens rather than whenever loop() next gets around to checking. That
 * could be a hand-written attachInterrupt() ISR, but on Teensy 3.x/4.x this
 * library instead uses a dedicated hardware quadrature-decoder timer where
 * available (falling back to interrupts otherwise) -- so ticks are counted
 * in hardware with zero CPU/ISR overhead. Practically: just construct an
 * Encoder object with the two pins and call .read(); you don't need to
 * write any interrupt code yourself.
 *
 * SAFETY: if no "M"/"S" command arrives within CMD_TIMEOUT_MS, the motor is
 * stopped automatically -- a dropped USB connection or a crashed Pi script
 * should never leave the motor running. NOTE: this means a controlling
 * script must keep re-sending the current command faster than
 * CMD_TIMEOUT_MS, not just send it once -- a single one-off command will
 * self-cancel in under half a second.
 *
 * LIBRARIES NEEDED (Arduino Library Manager): "Encoder" by Paul Stoffregen.
 *
 * STILL TO DO:
 *   - SABERTOOTH_S1_PIN -- double check it against your actual wiring.
 *   - PULSE_MIN_US/PULSE_NEUTRAL_US/PULSE_MAX_US -- the 1000/1500/2000us
 *     defaults match Sabertooth's standard R/C mode range, but confirm
 *     against your unit's manual/DIP switches if it seems off.
 */

#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>

// ============================== USER TUNABLES ===============================
constexpr int MOTOR1_ENC_A_PIN = 21;
constexpr int MOTOR1_ENC_B_PIN = 22;

// Sabertooth 2x12, R/C mode, motor 1 only for now -- CHANGE THIS PIN to match your wiring.
constexpr int SABERTOOTH_S1_PIN = 5;
// Standard R/C pulse range (matches Sabertooth's default R/C mode). Confirm
// against your unit's manual/DIP switches if it doesn't behave as expected.
constexpr int PULSE_MIN_US     = 1000;
constexpr int PULSE_NEUTRAL_US = 1500;
constexpr int PULSE_MAX_US     = 2000;

constexpr unsigned long CMD_TIMEOUT_MS    = 500;  // stop the motor if no command in this long
constexpr unsigned long ENCODER_STREAM_MS = 100;  // how often to send encoder ticks to the Pi

// To convert ticks_per_s into something physical once you know your specs:
//   rev_per_s = ticks_per_s / ENCODER_COUNTS_PER_REV   (counts-per-rev AFTER
//               the gearbox, i.e. at the output shaft -- check the datasheet,
//               this is usually NOT the same as the motor's raw CPR)
//   rad_per_s = rev_per_s * 2*PI
//   wheel_mps = rad_per_s * wheel_radius_m
// Left as a manual step for now since we don't have your encoder's specs.

// =============================================================================

// Quadrature decode happens in hardware on Teensy where available (else the
// library falls back to interrupts) -- .read() just returns the current
// accumulated tick count, no ISR code needed here.
Encoder motor1Encoder(MOTOR1_ENC_A_PIN, MOTOR1_ENC_B_PIN);
long lastEncCount = 0;
unsigned long lastEncStreamMs = 0;

// ------------------------------ motor output --------------------------------
Servo motor1;

// speed in [-1, 1]: -1 = full reverse, 0 = stop, +1 = full forward. Maps
// onto the R/C pulse width the Sabertooth expects; see the header comment
// for why this is Servo.writeMicroseconds() rather than analogWrite().
void setMotor1(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  int pulse_us = PULSE_NEUTRAL_US + (int)(speed * (PULSE_MAX_US - PULSE_NEUTRAL_US));
  motor1.writeMicroseconds(pulse_us);
}

void stopMotors() {
  setMotor1(0.0f);  // sends the neutral pulse -- the Sabertooth wants a live
                     // signal at all times in R/C mode, not silence.
}

// ------------------------------ serial protocol ------------------------------
float cmdSpeed = 0.0f;                   // motor 1 command, [-1, 1]
unsigned long lastCmdMs = 0;             // 0 at boot -> motor stays stopped until a real command arrives

void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line[0] == 'M' || line[0] == 'm') {
    int c1 = line.indexOf(',');
    if (c1 > 0) {
      cmdSpeed = line.substring(c1 + 1).toFloat();
      lastCmdMs = millis();
    } else {
      Serial.println(F("# bad command, expected M,<speed> (-1..1)"));
    }
  } else if (line[0] == 'S' || line[0] == 's') {
    cmdSpeed = 0.0f;
    lastCmdMs = millis();
    stopMotors();
  } else {
    Serial.println(F("# unrecognized command"));
  }
}

void setup() {
  Serial.begin(115200);  // Teensy's USB serial ignores the baud rate (always full USB speed) -- harmless to set anyway
  motor1.attach(SABERTOOTH_S1_PIN);  // must attach() before the first writeMicroseconds() below
  pinMode(MOTOR1_ENC_A_PIN, INPUT_PULLUP);
  pinMode(MOTOR1_ENC_B_PIN, INPUT_PULLUP);
  stopMotors();

  Serial.println(F("# teensy_bridge ready. Send M,<speed> (-1..1) or S"));
}

void loop() {
  handleSerialCommands();

  unsigned long nowMs = millis();
  if (nowMs - lastEncStreamMs >= ENCODER_STREAM_MS) {
    long count = motor1Encoder.read();
    float dt_s = (nowMs - lastEncStreamMs) / 1000.0f;
    float ticksPerSec = (count - lastEncCount) / dt_s;
    lastEncCount = count;
    lastEncStreamMs = nowMs;
    Serial.print(F("E,"));
    Serial.print(count);
    Serial.print(',');
    Serial.print(ticksPerSec, 2);
    Serial.print(',');
    Serial.println(nowMs);
  }

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopMotors();
  } else {
    setMotor1(cmdSpeed);
  }
}
