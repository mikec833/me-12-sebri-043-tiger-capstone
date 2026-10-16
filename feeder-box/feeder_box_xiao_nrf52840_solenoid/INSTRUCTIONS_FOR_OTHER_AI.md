# Instructions for the AI maintaining the feeder-box solenoid firmware

Use `feeder_box_xiao_nrf52840_solenoid.ino` with `latch_controller.h`. Do not merge
this with the earlier ESP32S3 sketch: the physical feeder board is now confirmed
as a Seeed XIAO nRF52840, so this firmware deliberately uses `bluefruit.h`.

Required interface contract:

- Relay `IN1` is connected to XIAO `D1`, not D2.
- Relay S1 trigger jumper shorts `COM` to `HIGH`; therefore HIGH means relay ON.
- BLE device name is `TigerBall-Latch`.
- Service UUID is `7b6a1000-6c3a-4c9f-ae60-62b79b0e5135`.
- Command characteristic UUID is `7b6a1001-6c3a-4c9f-ae60-62b79b0e5135`.
- The Raspberry Pi sends the exact ASCII command `OPEN` (the `command`
  parameter of `solenoid_release_node.py`, set under `solenoid_release_node`
  in `ros2_ws/src/ballrobot_pkg/config/*.yaml`). Do not change it here
  without changing those in the same revision.
- `OFF` must immediately de-energise the relay.
- One accepted `OPEN` energises the relay for 500 ms only. A command during
  an active pulse cannot extend it. The next pulse is rejected for 3 seconds
  after switch-off.

Failure behavior means the relay contact returns open and the solenoid loses
power. Preserve all four layers:

1. Set the D1 output latch LOW before calling `pinMode(D1, OUTPUT)`.
2. Use relay COM1 and NO1, never NC1, so loss of relay-coil power opens the load.
3. Call the OFF function on BLE disconnect and on an `OFF` command.
4. Keep the one-second nRF52840 hardware watchdog. If the main loop stalls, it
   resets the board and setup drives D1 LOW. This limits but does not eliminate
   the consequences of a software freeze; it cannot correct welded contacts or
   guarantee the mechanical latch position.

The user accepts an unauthenticated/open BLE write characteristic for this
prototype. Do not silently describe it as secure or suitable for animal-area
deployment.

Compile target used for verification:

`Seeeduino:nrf52:xiaonRF52840Sense`

Bench sequence: test D1/relay behavior first with the solenoid power branch
disconnected; confirm boot, reset, BLE disconnect, 500 ms timeout and cooldown;
then perform supervised loaded tests and measure the actual solenoid pulse.
