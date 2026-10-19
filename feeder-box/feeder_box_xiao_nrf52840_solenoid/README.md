# Corrected feeder-box solenoid firmware

Open `feeder_box_xiao_nrf52840_solenoid.ino` in Arduino IDE.

Select:

- Board package: **Seeed nRF52 Boards**
- Board: **Seeed XIAO nRF52840 Sense** (or the non-Sense XIAO nRF52840 if that
  is the exact physical board)

Then select the USB port and upload. This package was compiled for both the
Sense and non-Sense targets but was not flashed by Codex.

Wiring/protocol assumptions:

- XIAO D0 -> relay IN1
- S1 jumper -> COM-HIGH
- relay COM1/NO1 switch the solenoid supply
- Pi command -> `OPEN`
- BLE name -> `TigerBall-Latch`

Test first with the solenoid power branch disconnected. Confirm relay OFF at
boot, one approximately 500 ms click after `OPEN`, further `OPEN`s ignored for 10 s after it turns on,
OFF after BLE disconnect, and OFF after a deliberate watchdog-reset test.

See `INSTRUCTIONS_FOR_OTHER_AI.md` for the interface and safety invariants that
must be preserved in later edits.
