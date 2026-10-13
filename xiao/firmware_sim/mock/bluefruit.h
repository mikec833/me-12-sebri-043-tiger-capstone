#pragma once
#include "Arduino.h"
#define CHR_PROPS_NOTIFY 1
#define CHR_PROPS_WRITE 2
#define CHR_PROPS_WRITE_WO_RESP 4
#define SECMODE_OPEN 1
#define SECMODE_NO_ACCESS 0
#define BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE 6
extern std::string g_ble_out;
struct BLEService { BLEService(const char *) {} void begin() {} };
struct BLECharacteristic {
  typedef void (*wcb)(uint16_t, BLECharacteristic *, uint8_t *, uint16_t);
  BLECharacteristic(const char *) {}
  void setProperties(int) {} void setPermission(int, int) {} void setMaxLen(int) {} void begin() {}
  void setWriteCallback(wcb) {}
  bool notifyEnabled() { return true; }
  bool notify(const char *d, size_t n) { g_ble_out.append(d, n); return true; }
};
struct Periph_ { void setConnectCallback(void (*)(uint16_t)) {} void setDisconnectCallback(void (*)(uint16_t, uint8_t)) {} };
struct Adv_ { void addFlags(int) {} void addTxPower() {} void addService(BLEService &) {} void restartOnDisconnect(bool) {}
              void setInterval(int, int) {} void setFastTimeout(int) {} void start(int) {} };
struct Scan_ { void addName() {} };
struct Bluefruit_ { Periph_ Periph; Adv_ Advertising; Scan_ ScanResponse; void begin(int, int) {} void setName(const char *) {} };
extern Bluefruit_ Bluefruit;
