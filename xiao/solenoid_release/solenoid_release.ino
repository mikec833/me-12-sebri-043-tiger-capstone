// solenoid_release.ino
// Feeder-box solenoid lock release on a Seeed XIAO ESP32-S3, commanded
// over Bluetooth Low Energy by solenoid_release_node.py on the Pi. On a valid
// RELEASE the XIAO closes a relay for PULSE_MS, which puts a short 12 V pulse
// across the solenoid lock and pops the feeder box.
//
// BLE interface (GATT server, advertises as DEVICE_NAME):
//   Service  SERVICE_UUID
//     CMD_UUID     write    "RELEASE:<TOKEN>" -> pulses the relay once.
//                                                Repeats after that do nothing, so
//                                                the Pi can retry blindly.
//                           "ARM:<TOKEN>"     -> re-arms for another release
//                                                (bench testing without a power cycle).
//     STATUS_UUID  read     result of the last command:
//                           "armed" | "released" | "already_released" | "bad_token" | "bad_cmd"
//   The Pi writes a command, then reads STATUS to confirm it actually fired.
//
// Board:  XIAO ESP32S3 (Tools -> Board -> XIAO_ESP32S3), arduino-esp32 core (2.0.x
//         or 3.x), built-in BLE library -- nothing extra to install. Screw on the
//         external antenna that ships with it; it sets how far away the Pi can connect.
//         (Also builds for a XIAO ESP32C3.)
// Power: wall -> buck converter -> 5 V out (XIAO + relay module) and 12 V out (lock).
// Wiring (relay module = board with its own transistor/optocoupler, not a bare relay):
//   buck 5 V  -> XIAO 5V pin, relay VCC (5 V module; a 5 V module may not trigger
//                reliably from 3.3 V logic on IN -- check it clicks on the bench)
//   XIAO D1   -> relay IN
//   buck 12 V -> relay COM
//   relay NO  -> solenoid lock +
//   solenoid lock - -> buck GND
//   All grounds common: buck GND, XIAO GND, relay GND.
//   Use NO (normally open) so the lock is unpowered if the XIAO is off or resets.
//   The XIAO never touches the 12 V side; only the relay contacts do.
//   No flyback diode fitted: the lock's switch-off spike arcs across the relay
//   contacts. Keep the lock's wires short and away from the XIAO. If the spike
//   resets the XIAO it only happens after the pulse, and the Pi has already
//   latched the release by then, so it won't fire twice.
//   Don't power the XIAO from USB and the buck 5 V at the same time.
//
// Usage: flash, open Serial Monitor at 115200. It prints its BLE address on
//        boot; the Pi finds it by DEVICE_NAME by default, or pass that address
//        as the node's xiao_address parameter to pin it to this board.

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// ---------------- BLE ----------------
const char* DEVICE_NAME = "feeder-solenoid";
// Must match the Pi node. Any UUIDs work as long as both sides agree.
#define SERVICE_UUID "6e0f0001-3b5a-4c2e-9d1a-2f5c8e7b4a10"
#define CMD_UUID     "6e0f0002-3b5a-4c2e-9d1a-2f5c8e7b4a10"
#define STATUS_UUID  "6e0f0003-3b5a-4c2e-9d1a-2f5c8e7b4a10"

// Shared secret with the Pi node's `token` parameter, so another BLE device
// nearby can't fire the solenoid just by finding the characteristic.
const char* TOKEN = "tiger-feeder";

// ---------------- Relay ----------------
// D1 is GPIO3 on the C3 and GPIO2 on the S3 -- neither is a boot strapping pin,
// so the relay won't click during reset/flash.
// D1 is only defined when a XIAO board is selected in Tools -> Board; with a
// generic ESP32C3/S3 board, fall back to the same physical pin by GPIO number.
#if defined(D1)
constexpr int RELAY_PIN = D1;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
constexpr int RELAY_PIN = 3;   // XIAO ESP32C3 D1
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
constexpr int RELAY_PIN = 2;   // XIAO ESP32S3 D1
#else
#error "Select XIAO_ESP32C3 or XIAO_ESP32S3 under Tools -> Board"
#endif
// Many cheap relay modules (the blue SRD-05VDC boards) switch ON when IN is LOW.
// If the relay clicks on at boot and off on a release, flip this.
constexpr bool RELAY_ACTIVE_HIGH = true;
// How long the 12 V is applied. Solenoid locks are rated for short duty (often
// <10 s on), so keep it brief -- just long enough for the latch to clear and the
// lid to start opening. Tune on the bench.
constexpr uint32_t PULSE_MS = 500;

BLECharacteristic* statusChar = nullptr;

// Written from the BLE task (onWrite), read in loop().
volatile bool armed = true;
volatile bool pulsing = false;
volatile uint32_t pulseStartMs = 0;
volatile uint32_t releaseCount = 0;
volatile bool restartAdvertising = false;

void relayWrite(bool on) {
  digitalWrite(RELAY_PIN, (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

void setStatus(const char* s) {
  statusChar->setValue(s);
  Serial.printf("# status -> %s\n", s);
}

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    // getValue() is std::string on core 2.x and String on 3.x; c_str() works for both.
    String v = String(c->getValue().c_str());
    v.trim();
    int sep = v.indexOf(':');
    String cmd = (sep < 0) ? v : v.substring(0, sep);
    String tok = (sep < 0) ? String("") : v.substring(sep + 1);

    if (tok != TOKEN) {
      setStatus("bad_token");
      return;
    }
    if (cmd == "RELEASE") {
      if (!armed) {
        setStatus("already_released");
        return;
      }
      armed = false;
      pulseStartMs = millis();
      pulsing = true;
      releaseCount++;
      relayWrite(true);
      setStatus("released");
    } else if (cmd == "ARM") {
      armed = true;
      setStatus("armed");
    } else {
      setStatus("bad_cmd");
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    Serial.println("# central connected");
  }
  void onDisconnect(BLEServer*) override {
    // Advertising stops on connect; restart it from loop() so the Pi can reconnect.
    restartAdvertising = true;
    Serial.println("# central disconnected");
  }
};

void setup() {
  // Relay off before anything else so a reset can never leave the lock energised.
  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false);

  Serial.begin(115200);
  delay(500);

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_P9);  // max TX power for range

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);
  BLECharacteristic* cmdChar = service->createCharacteristic(
      CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmdChar->setCallbacks(new CmdCallbacks());
  statusChar = service->createCharacteristic(
      STATUS_UUID, BLECharacteristic::PROPERTY_READ);
  statusChar->setValue("armed");
  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);  // room for the full name alongside the 128-bit UUID
  BLEDevice::startAdvertising();

  Serial.printf("# advertising as \"%s\", address %s\n",
                DEVICE_NAME, BLEDevice::getAddress().toString().c_str());
}

void loop() {
  // End the pulse non-blocking so BLE keeps answering retries meanwhile.
  if (pulsing && millis() - pulseStartMs >= PULSE_MS) {
    relayWrite(false);
    pulsing = false;
    Serial.printf("# pulse done (releases = %lu)\n", (unsigned long)releaseCount);
  }

  if (restartAdvertising) {
    restartAdvertising = false;
    delay(100);  // let the stack finish tearing down the old connection
    BLEDevice::startAdvertising();
    Serial.println("# advertising again");
  }
}
