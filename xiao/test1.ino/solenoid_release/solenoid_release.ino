#include <bluefruit.h>
// nrf chip
// --------------------------------------------------
// Relay settings
// --------------------------------------------------

// XIAO D1 connects directly to relay IN1.
// S1 jumper must connect COM and HIGH.
constexpr uint8_t RELAY_PIN = D1;
constexpr uint8_t RELAY_ON  = HIGH;
constexpr uint8_t RELAY_OFF = LOW;

constexpr uint32_t PULSE_TIME_MS    = 500;
constexpr uint32_t COOLDOWN_TIME_MS = 3000;

// --------------------------------------------------
// BLE settings
// --------------------------------------------------

const char* DEVICE_NAME = "TigerBall-Latch";

const char* SERVICE_UUID =
  "7b6a1000-6c3a-4c9f-ae60-62b79b0e5135";

const char* COMMAND_UUID =
  "7b6a1001-6c3a-4c9f-ae60-62b79b0e5135";

BLEService latchService(SERVICE_UUID);
BLECharacteristic commandCharacteristic(COMMAND_UUID);

// --------------------------------------------------
// Relay state
// --------------------------------------------------

bool relayIsOn = false;
bool pulsePreviouslyCompleted = false;

uint32_t relayOffTime = 0;
uint32_t lastPulseFinishedTime = 0;

// --------------------------------------------------
// Relay functions
// --------------------------------------------------

void switchRelayOff()
{
  digitalWrite(RELAY_PIN, RELAY_OFF);

  if (relayIsOn)
  {
    lastPulseFinishedTime = millis();
    pulsePreviouslyCompleted = true;
  }

  relayIsOn = false;
}

void startRelayPulse()
{
  uint32_t currentTime = millis();

  // Do not extend an active pulse
  if (relayIsOn)
  {
    return;
  }

  // Enforce cooldown between pulses
  if (
    pulsePreviouslyCompleted &&
    (currentTime - lastPulseFinishedTime < COOLDOWN_TIME_MS)
  )
  {
    return;
  }

  digitalWrite(RELAY_PIN, RELAY_ON);

  relayIsOn = true;
  relayOffTime = currentTime + PULSE_TIME_MS;
}

// --------------------------------------------------
// BLE write callback
// --------------------------------------------------

void commandWriteCallback(
  uint16_t connHandle,
  BLECharacteristic* characteristic,
  uint8_t* data,
  uint16_t len
)
{
  (void)connHandle;
  (void)characteristic;

  // Copy received BLE data into a null-terminated string
  char buffer[32];

  uint16_t copyLength = len;

  if (copyLength >= sizeof(buffer))
  {
    copyLength = sizeof(buffer) - 1;
  }

  memcpy(buffer, data, copyLength);
  buffer[copyLength] = '\0';

  String command = String(buffer);

  command.trim();
  command.toUpperCase();

  if (command == "PULSE" || command == "5")
  {
    startRelayPulse();
  }
  else if (command == "OFF" || command == "0")
  {
    switchRelayOff();
  }
}

// --------------------------------------------------
// BLE disconnect callback
// --------------------------------------------------

void disconnectCallback(uint16_t connHandle, uint8_t reason)
{
  (void)connHandle;
  (void)reason;

  // Fail safe: relay OFF if Bluetooth disconnects
  switchRelayOff();
}

// --------------------------------------------------
// Advertising
// --------------------------------------------------

void startAdvertising()
{
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE
  );

  Bluefruit.Advertising.addTxPower();

  // Advertise our custom service
  Bluefruit.Advertising.addService(latchService);

  // Put device name in scan response
  Bluefruit.ScanResponse.addName();

  // Automatically resume advertising after disconnect
  Bluefruit.Advertising.restartOnDisconnect(true);

  // Fast / slow advertising intervals
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);

  // 0 = advertise indefinitely
  Bluefruit.Advertising.start(0);
}

// --------------------------------------------------
// Setup
// --------------------------------------------------

void setup()
{
  // Relay must start OFF
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  // Initialise BLE
  Bluefruit.begin();

  Bluefruit.setName(DEVICE_NAME);

  // Disconnect callback
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // Start custom service
  latchService.begin();

  // Configure command characteristic
  commandCharacteristic.setProperties(
    CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP
  );

  commandCharacteristic.setPermission(
    SECMODE_NO_ACCESS,
    SECMODE_OPEN
  );

  commandCharacteristic.setMaxLen(20);

  commandCharacteristic.setWriteCallback(
    commandWriteCallback
  );

  commandCharacteristic.begin();

  // Start BLE advertising
  startAdvertising();
}

// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop()
{
  // Automatically switch relay OFF after 500 ms
  if (
    relayIsOn &&
    static_cast<int32_t>(millis() - relayOffTime) >= 0
  )
  {
    switchRelayOff();
  }

  delay(2);
}