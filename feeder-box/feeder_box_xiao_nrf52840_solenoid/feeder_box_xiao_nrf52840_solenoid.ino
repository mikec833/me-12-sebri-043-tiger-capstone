#include <bluefruit.h>
#include <nrf.h>

#include "latch_controller.h"

// Feeder-box controller for Seeed XIAO nRF52840 / nRF52840 Sense.
// Pi command: ASCII "OPEN" over BLE.
// Relay wiring: XIAO D1 -> relay IN1; S1 jumper COM-HIGH.

constexpr uint8_t RELAY_PIN = D1;
constexpr uint8_t RELAY_ON = HIGH;
constexpr uint8_t RELAY_OFF = LOW;

constexpr uint32_t PULSE_TIME_MS = 500;
constexpr uint32_t COOLDOWN_TIME_MS = 3000;

// nRF52840 watchdog clock is 32768 Hz. If loop() stops feeding it for about
// one second, hardware resets the XIAO; setup() then drives the relay OFF.
constexpr uint32_t WATCHDOG_RELOAD_TICKS = 32768;

const char* DEVICE_NAME = "TigerBall-Latch";
const char* SERVICE_UUID = "7b6a1000-6c3a-4c9f-ae60-62b79b0e5135";
const char* COMMAND_UUID = "7b6a1001-6c3a-4c9f-ae60-62b79b0e5135";

BLEService latchService(SERVICE_UUID);
BLECharacteristic commandCharacteristic(COMMAND_UUID);
LatchController latch(PULSE_TIME_MS, COOLDOWN_TIME_MS);

void setRelayOff() {
  digitalWrite(RELAY_PIN, RELAY_OFF);
  latch.forceOff(millis());
}

void requestLatchPulse() {
  if (latch.requestPulse(millis())) {
    digitalWrite(RELAY_PIN, RELAY_ON);
  }
}

void startHardwareWatchdog() {
  // The nRF watchdog cannot be stopped after it starts. Pause only while a
  // debugger halts the CPU; keep it running during normal CPU sleep.
  if (NRF_WDT->RUNSTATUS == 0) {
    NRF_WDT->CONFIG =
        (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
        (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
    NRF_WDT->CRV = WATCHDOG_RELOAD_TICKS;
    NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
    NRF_WDT->TASKS_START = 1;
  }
}

void feedHardwareWatchdog() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

void commandWriteCallback(uint16_t connHandle,
                          BLECharacteristic* characteristic,
                          uint8_t* data,
                          uint16_t len) {
  (void)connHandle;
  (void)characteristic;

  char buffer[32];
  uint16_t copyLength = len;
  if (copyLength >= sizeof(buffer)) {
    copyLength = sizeof(buffer) - 1;
  }

  memcpy(buffer, data, copyLength);
  buffer[copyLength] = '\0';

  String command(buffer);
  command.trim();
  command.toUpperCase();

  // Must match the `command` parameter of solenoid_release_node.py
  // (set under solenoid_release_node in the ros2_ws config YAMLs).
  if (command == "OPEN") {
    requestLatchPulse();
  } else if (command == "OFF") {
    setRelayOff();
  }
}

void disconnectCallback(uint16_t connHandle, uint8_t reason) {
  (void)connHandle;
  (void)reason;

  // Failure state: relay contact open and solenoid de-energised.
  setRelayOff();
}

void startAdvertising() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(latchService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setup() {
  // Load the OFF value before enabling the output to avoid an ON transition
  // during setup. The relay's NO contact must be used for the solenoid.
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pinMode(RELAY_PIN, OUTPUT);

  Bluefruit.begin();
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  latchService.begin();

  commandCharacteristic.setProperties(
      CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  commandCharacteristic.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  commandCharacteristic.setMaxLen(20);
  commandCharacteristic.setWriteCallback(commandWriteCallback);
  commandCharacteristic.begin();

  startAdvertising();

  // Start only after BLE setup, so startup cannot trip the watchdog.
  startHardwareWatchdog();
  feedHardwareWatchdog();
}

void loop() {
  if (latch.update(millis())) {
    digitalWrite(RELAY_PIN, RELAY_OFF);
  }

  feedHardwareWatchdog();
  delay(2);
}
