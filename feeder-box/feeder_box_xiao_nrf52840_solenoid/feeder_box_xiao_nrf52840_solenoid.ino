#include <bluefruit.h>

// Feeder-box controller for Seeed XIAO nRF52840.
// The Pi's solenoid_release_node.py writes "OPEN" over BLE when
// /feeder_box_threshold_reached goes True. The relay on D0 turns on for
// 500 ms, then further OPENs are ignored for 10 s. "OFF" turns it off now.

const int RELAY_PIN = D0;
const unsigned long PULSE_MS = 500;
const unsigned long TIMEOUT_MS = 10000;

// Must match solenoid_release_node.py on the Pi
const char* DEVICE_NAME = "TigerBall-Latch";
BLEService latchService("7b6a1000-6c3a-4c9f-ae60-62b79b0e5135");
BLECharacteristic commandChar("7b6a1001-6c3a-4c9f-ae60-62b79b0e5135");

bool relayOn = false;
bool hasOpened = false;
unsigned long openedAt = 0;

void relayOff() {
  digitalWrite(RELAY_PIN, LOW);
  relayOn = false;
}

void openFeeder() {
  if (relayOn) return;
  if (hasOpened && millis() - openedAt < TIMEOUT_MS) return;  // still in timeout

  digitalWrite(RELAY_PIN, HIGH);
  relayOn = true;
  hasOpened = true;
  openedAt = millis();
}

void onCommand(uint16_t conn, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  String cmd = "";
  for (int i = 0; i < len; i++) cmd += (char)data[i];
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "OPEN") openFeeder();
  else if (cmd == "OFF") relayOff();
}

void onDisconnect(uint16_t conn, uint8_t reason) {
  relayOff();
}

void setup() {
  digitalWrite(RELAY_PIN, LOW);  // set LOW before enabling output so the relay never blips on
  pinMode(RELAY_PIN, OUTPUT);

  Bluefruit.begin();
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  latchService.begin();
  commandChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  commandChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  commandChar.setMaxLen(20);
  commandChar.setWriteCallback(onCommand);
  commandChar.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(latchService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);

  // Watchdog: if loop() hangs for ~1 s the board resets and setup() turns the relay off
  NRF_WDT->CRV = 32768;
  NRF_WDT->RREN = WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}

void loop() {
  if (relayOn && millis() - openedAt >= PULSE_MS) relayOff();

  NRF_WDT->RR[0] = WDT_RR_RR_Reload;  // feed watchdog
  delay(2);
}
