#include "Arduino.h"
#include "bluefruit.h"
uint32_t g_millis = 1000;
MockSerial Serial;
std::string g_ble_out;
Bluefruit_ Bluefruit;
#include "../TigerBall_nRF52840_Sense_v4_7/TigerBall_nRF52840_Sense_v4_7.ino"

static int fails = 0, checks = 0;
#define CHECK(cond, what) do { ++checks; if (!(cond)) { ++fails; printf("  FAIL %s  (%s:%d)\n", what, __FILE__, __LINE__); } } while (0)
static bool wheelsFree = true;   // encoders follow applied output unless a stall is simulated

static void step(uint32_t ms) {
  for (uint32_t i = 0; i < ms; ++i) {
    ++g_millis;
    if (wheelsFree) { leftCount += (int32_t)lroundf(appliedLeft * 0.5f); rightCount -= (int32_t)lroundf(appliedRight * 0.5f); }
    loop();
  }
}
static void serial(const char *s) { Serial.feed(s); }
static void ble(const char *s) { onRxWrite(0, nullptr, (uint8_t *)s, (uint16_t)strlen(s)); }
static bool has(const std::string &buf, const char *s) { return buf.find(s) != std::string::npos; }
static void clearOut() { Serial.out.clear(); g_ble_out.clear(); }
static void reset() {
  ble("K\n"); serial("K\n"); step(5);
  if (stallFault) { serial("R\n"); step(5); }
  Serial.open = true; wheelsFree = true; step(50); clearOut();
}
static void serialDrive(const char *m, uint32_t ms) {  // M at 10 Hz
  for (uint32_t t = 0; t < ms; t += 100) { serial(m); step(100); }
}
static void serialArm(int maxPct) {
  char c[32]; snprintf(c, sizeof c, "C,800,700,%d\n", maxPct); serial(c); serial("A\n"); step(5);
}

int main() {
  setup();
  bleConnected = true; onBleConnect(0); step(10); clearOut();

  printf("V: per-source version\n");
  ble("V\n"); serial("V\n"); step(2);
  CHECK(has(g_ble_out, "VERSION,4.6\n") && !has(g_ble_out, "4.7"), "BLE gets VERSION,4.6 only");
  CHECK(has(Serial.out, "VERSION,4.7\n"), "serial gets VERSION,4.7");
  reset();

  printf("F2: serial only, watchdog\n");
  serialArm(20);
  CHECK(armed && controlOwner == SRC_SERIAL, "armed under SERIAL");
  CHECK(has(Serial.out, "STATE,ARMED\n") && has(Serial.out, "OWNER,SERIAL\n"), "serial gets ARMED + OWNER");
  CHECK(has(g_ble_out, "OWNER,SERIAL\n") && !has(g_ble_out, "STATE,ARMED"), "BLE gets OWNER only");
  CHECK(configuredMaximumPercent == 20.0f, "serial C applied on arm");
  serialDrive("M,15,15\n", 2000);
  CHECK(appliedLeft > 10.0f && appliedRight > 10.0f, "drives forward");
  clearOut();
  // serialDrive returns 100 ms after its last M
  step(350); CHECK(targetLeft > 0.0f, "targets held at 0.45 s");
  step(100); CHECK(targetLeft == 0.0f && targetRight == 0.0f && armed, "soft stop at 0.55 s, still armed");
  step(600); CHECK(armed, "still armed at 1.15 s");
  step(100); CHECK(!armed && controlOwner == SRC_NONE, "disarmed at 1.25 s");
  CHECK(has(Serial.out, "STATE,TIMEOUT_DISARMED\n") && has(g_ble_out, "STATE,TIMEOUT_DISARMED\n"), "TIMEOUT to both");
  CHECK(has(Serial.out, "OWNER,NONE\n"), "OWNER,NONE broadcast");
  reset();

  printf("F3/F4: serial driving, website connects and sends M\n");
  serialArm(20); serialDrive("M,15,15\n", 1000); clearOut();
  onBleDisconnect(0, 0); step(1); onBleConnect(0); step(1);
  CHECK(armed && controlOwner == SRC_SERIAL, "BLE reconnect does not disarm a serial run");
  CHECK(has(g_ble_out, "OWNER,SERIAL\n"), "website told OWNER,SERIAL");
  const float tl = targetLeft;
  ble("M,40,40\n"); step(1);
  CHECK(targetLeft == tl, "website M ignored while serial owns");
  serialDrive("M,15,15\n", 300);

  printf("F5/F6: keeper takeover\n");
  clearOut(); ble("C,800,700,30\n"); ble("A\n"); step(2);
  CHECK(armed && controlOwner == SRC_BLE, "BLE A takes over");
  CHECK(targetLeft == 0.0f && targetRight == 0.0f, "takeover stops the robot");
  CHECK(has(g_ble_out, "OWNER,BLE\n") && has(Serial.out, "OWNER,BLE\n"), "OWNER,BLE to both");
  CHECK(!has(Serial.out, "OWNER,NONE"), "no spurious OWNER,NONE on takeover");
  CHECK(configuredMaximumPercent == 30.0f, "BLE arm uses BLE config, not serial's");
  clearOut(); serial("M,15,15\n"); step(2);
  CHECK(targetLeft == 0.0f, "serial M ignored after takeover");
  CHECK(has(Serial.out, "STATE,NOT_OWNER\n"), "serial told NOT_OWNER");
  clearOut(); serial("A\n"); step(2);
  CHECK(has(Serial.out, "STATE,BUSY,BLE\n") && controlOwner == SRC_BLE, "serial A refused: BUSY,BLE");
  serial("C,800,700,90\n"); step(2);
  CHECK(configuredMaximumPercent == 30.0f, "serial C not applied live while BLE owns");
  reset();

  printf("Non-owner heartbeat cannot keep the robot alive\n");
  serialArm(20); serialDrive("M,15,15\n", 500);
  for (int i = 0; i < 20; ++i) { ble("S\n"); step(100); }
  CHECK(!armed, "BLE S heartbeats do not feed the serial watchdog");
  reset();

  printf("F7: website K stops a serial run\n");
  serialArm(20); serialDrive("M,15,15\n", 500); clearOut();
  ble("K\n"); step(2);
  CHECK(!armed && controlOwner == SRC_NONE, "disarmed");
  CHECK(has(g_ble_out, "STATE,DISARMED\n") && has(Serial.out, "STATE,DISARMED\n"), "DISARMED to both");
  CHECK(has(Serial.out, "OWNER,NONE\n"), "OWNER,NONE to serial");
  reset();

  printf("Website S stops a serial run\n");
  serialArm(20); serialDrive("M,15,15\n", 500);
  ble("S\n"); step(1);
  CHECK(targetLeft == 0.0f && targetRight == 0.0f, "BLE S zeroes targets");
  reset();

  printf("F8: website driving, website disconnects\n");
  ble("C,800,700,20\n"); ble("A\n"); step(2);
  for (int i = 0; i < 5; ++i) { ble("M,15,15\n"); step(100); }
  onBleDisconnect(0, 0); step(1);
  CHECK(!armed && controlOwner == SRC_NONE, "disarmed on disconnect (v4.6)");
  onBleConnect(0); step(1); reset();

  printf("F9: Pi closes the port\n");
  serialArm(20); serialDrive("M,15,15\n", 500);
  Serial.open = false; step(1);
  CHECK(!armed && controlOwner == SRC_NONE, "host loss disarms immediately");
  reset();

  printf("F10: stall during serial run, website R\n");
  serialArm(20); wheelsFree = false; serialDrive("M,15,15\n", 2500); clearOut();
  CHECK(stallFault && !armed, "stall fault trips");
  serialArm(20);
  CHECK(!armed && has(Serial.out, "FAULT,ENCODER_STALL\n"), "serial A refused during fault, told why");
  wheelsFree = true; clearOut(); ble("R\n"); step(2);
  CHECK(!stallFault, "website R clears the fault");
  CHECK(has(Serial.out, "STATE,FAULT_RESET\n") && has(g_ble_out, "STATE,FAULT_RESET\n"), "FAULT_RESET to both");
  reset();

  printf("F11: 100-char serial line while serial owns\n");
  serialArm(20); serialDrive("M,15,15\n", 300);
  std::string longline(100, '9'); longline += "\nM,15,15\n"; serial(longline.c_str()); step(5);
  CHECK(!armed && appliedLeft == 0.0f, "overflow disarms, robot neutral");
  CHECK(targetLeft == 0.0f, "the M after the overflow is not obeyed while disarmed");
  reset();

  printf("F12: legacy REF lines\n");
  serialArm(20); serialDrive("M,12,12\n", 300); clearOut();
  const float before = targetLeft;
  serial("REF,1.000,2.000\n"); step(2);
  CHECK(armed && targetLeft == before && !has(Serial.out, "NOT_OWNER"), "REF ignored, no effect, no reply");
  reset();

  printf("Partial serial line is not processed without a terminator\n");
  serialArm(20); serial("K"); step(20);
  CHECK(armed, "unterminated serial K not acted on");
  serial("\n"); step(2); CHECK(!armed, "acted on once terminated");
  reset();

  printf("BLE write without terminator is one command (v4.6 rule)\n");
  serialArm(20); ble("K"); step(2);
  CHECK(!armed, "unterminated BLE K acted on");
  reset();

  printf("BLE queue overflow is treated as S\n");
  serialArm(20); serialDrive("M,15,15\n", 300);
  for (int i = 0; i < 9; ++i) ble("V\n");   // 9 lines before loop() runs; depth is 8
  step(1);
  CHECK(targetLeft == 0.0f && targetRight == 0.0f, "queue full stops the robot");
  reset();

  printf("BLE 81-char line disarms (v4.6)\n");
  serialArm(20); std::string bl(85, '1'); bl += "\n"; ble(bl.c_str()); step(2);
  CHECK(!armed, "BLE overflow disarms");
  reset();

  printf("Every disarm resets both stored maxima\n");
  ble("C,800,700,50\n"); serial("C,800,700,60\n"); step(2); ble("K\n"); step(2);
  serial("A\n"); step(2);
  CHECK(armed && configuredMaximumPercent == 10.0f, "arm after disarm without fresh C runs at 10 %");
  reset();

  printf("F13: flood both sources for 60 s\n");
  serialArm(20); clearOut();
  uint32_t start = g_millis;
  for (int t = 0; t < 60000; t += 10) {
    serial("M,15,15\n"); ble("M,30,30\n"); ble("S\n");
    step(10);
  }
  size_t meas = 0; for (size_t p = 0; (p = Serial.out.find("MEAS,", p)) != std::string::npos; ++p) ++meas;
  CHECK(meas > 5900 && meas <= 6001, "MEAS still at 100 Hz");
  CHECK(armed && controlOwner == SRC_SERIAL, "serial run survives the flood");
  printf("  MEAS lines in 60 s: %zu (%u ms)\n", meas, g_millis - start);

  printf("\n%d/%d checks passed\n", checks - fails, checks);
  return fails ? 1 : 0;
}
