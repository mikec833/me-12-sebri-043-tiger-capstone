#!/usr/bin/env python3
"""
pi_bridge.py
------------------------------------------------------------------------------
Integration/bring-up script for today's Pi + Teensy + IMU setup:
  - Talks to the Teensy over USB-serial (Teensy runs
    firmware/teensy_bridge/teensy_bridge.ino, which streams encoder ticks and
    accepts a motor 1 speed command over serial -- it has no IMU code at all
    now that the BNO085 is wired directly to the Pi instead).
  - Reads the BNO085 directly over the Pi's own UART via bno085_rvc.py
    (separate physical connection from the Teensy -- GPIO14/15, not USB).
  - Leaves a clearly marked slot for the UWB pose reader -- your colleague's
    script goes in get_uwb_pose() below. Everything else is ready for it.
  - Continuously prints heading/encoder/pose to the terminal while letting
    you type a motor speed command at any time -- typing one doesn't
    interrupt the live feed, and the command keeps holding (see the
    CMD_TIMEOUT_MS note below) until you change it.

Only one motor (S1) is wired up right now, so commands are a single
normalized speed in [-1, 1], not a robot-level v/w -- there's no second
motor yet for that to mean anything. See teensy_bridge.ino's header comment
for the plan to bring differential-drive v/w control back once S2 is wired.

IMPORTANT: teensy_bridge.ino stops the motor if it doesn't receive a fresh
command within 500ms (CMD_TIMEOUT_MS) -- a safety watchdog so a crashed
script or unplugged cable can't leave a motor running forever. That means a
single one-off send isn't enough to hold a speed; this script runs a
background thread that keeps re-sending your last commanded speed every
150ms so it actually holds until you type a new one.

Wiring:
  - Teensy connected to the Pi via micro-USB -> USB-A. Run `ls /dev/ttyACM*`
    to confirm the exact name (usually /dev/ttyACM0); adjust SERIAL_PORT
    below if it differs.
  - BNO085 wired directly to the Pi's UART (GPIO14/15, /dev/serial0). See
    bno085_rvc.py's header comment for the one-time `raspi-config` UART
    setup this needs, and for how to sanity-check it standalone
    (`python3 bno085_rvc.py`) before running this script.
Requires: pip install pyserial

Usage:
    python3 pi_bridge.py
    (heading/encoder/pose lines start streaming immediately)
    0.2       <- type this + Enter: drives motor 1 forward at 20%, holds
    -0.2      <- reverse at 20%
    stop      <- stop
    q         <- quit
"""
import sys
import threading
import time

import serial

from bno085_rvc import BNO085RVCReader

TEENSY_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200  # Teensy's USB-CDC serial actually ignores this and always runs at full USB
                     # speed -- kept here for clarity / in case you swap to a UART-based board later.

KEEPALIVE_INTERVAL_S = 0.15  # well under the Teensy's 500ms CMD_TIMEOUT_MS watchdog -- see note above
STATUS_PRINT_INTERVAL_S = 0.5

encoder_lock = threading.Lock()
latest_enc_ticks = None
latest_enc_ticks_per_s = None
latest_enc_ts = 0.0

command_lock = threading.Lock()
current_speed = 0.0


def get_uwb_pose():
    """TODO: wire in your colleague's UWB script here. Return (x, y, z) in
    meters, or None if no fix is available yet. Left as a stub for today --
    everything else in this script (serial link, teleop, terminal display)
    is ready for it to slot straight in; see status_line() below for where
    it gets printed."""
    return None


def teensy_reader(ser):
    """Background thread: continuously parses encoder lines from the Teensy
    ('E,<ticks>,<ticks_per_s>,<millis>') and stores the latest value."""
    global latest_enc_ticks, latest_enc_ticks_per_s, latest_enc_ts
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except serial.SerialException:
            print("\n# serial read error -- Teensy disconnected?", file=sys.stderr)
            return
        if not line:
            continue
        if line.startswith("E,"):
            parts = line.split(",")
            if len(parts) >= 3:
                try:
                    with encoder_lock:
                        latest_enc_ticks = int(parts[1])
                        latest_enc_ticks_per_s = float(parts[2])
                        latest_enc_ts = time.monotonic()
                except ValueError:
                    pass
        elif line.startswith("#"):
            print(line)  # pass through Teensy status/debug messages


def command_sender(ser, stop_event):
    """Keeps re-sending the currently commanded speed so it doesn't get cut
    off by the Teensy's 500ms command-timeout watchdog."""
    while not stop_event.is_set():
        with command_lock:
            speed = current_speed
        try:
            ser.write(f"M,{speed:.4f}\n".encode("ascii"))
        except serial.SerialException:
            return
        time.sleep(KEEPALIVE_INTERVAL_S)


def status_line(imu):
    yaw, _pitch, _roll, age_s = imu.latest()
    if yaw is None:
        heading_str = "no IMU data yet"
    else:
        heading_str = f"yaw={yaw:+.2f} deg ({age_s * 1000:.0f} ms old)"

    with encoder_lock:
        ticks, rate = latest_enc_ticks, latest_enc_ticks_per_s
    enc_str = "no encoder data yet" if ticks is None else f"{ticks} ticks ({rate:+.1f} ticks/s)"

    pose = get_uwb_pose()  # <-- pose prints here automatically once get_uwb_pose() is wired in
    pose_str = "pending UWB integration" if pose is None else f"{pose}"

    return f"heading: {heading_str}   |   encoder: {enc_str}   |   pose: {pose_str}"


def status_printer(imu, stop_event):
    """Background thread: prints the live status line on its own cadence so
    it keeps updating even while you're not typing anything."""
    while not stop_event.is_set():
        print(status_line(imu))
        time.sleep(STATUS_PRINT_INTERVAL_S)


def main():
    global current_speed

    ser = serial.Serial(TEENSY_PORT, BAUD_RATE, timeout=0.2)
    # Brief pause so the first few reads are stable. Note: unlike an Uno,
    # the Teensy does NOT reboot when a serial connection opens, so this is
    # just settling time, not a reset wait.
    time.sleep(1.0)

    imu = BNO085RVCReader().start()

    stop_event = threading.Event()
    threading.Thread(target=teensy_reader, args=(ser,), daemon=True).start()
    threading.Thread(target=command_sender, args=(ser, stop_event), daemon=True).start()
    threading.Thread(target=status_printer, args=(imu, stop_event), daemon=True).start()

    print(f"# connected to Teensy on {TEENSY_PORT} and IMU on {imu.ser.port}.")
    print("# streaming heading/encoder/pose below. Type a speed in [-1, 1] + Enter "
          "to drive motor 1 (it holds until you change it), 'stop', or 'q' to quit.\n")
    try:
        while True:
            try:
                line = input().strip()
            except EOFError:
                break
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break
            if line in ("stop", "s"):
                with command_lock:
                    current_speed = 0.0
                continue
            try:
                speed = float(line)
            except ValueError:
                print("# unrecognized -- type a speed (e.g. '0.2' or '-0.2'), 'stop', or 'q'")
                continue
            if abs(speed) > 1.0:
                print("# speed should be in [-1, 1] -- sending anyway, the Teensy will clamp it")
            with command_lock:
                current_speed = speed
    finally:
        stop_event.set()
        with command_lock:
            current_speed = 0.0
        try:
            ser.write(b"S\n")
            time.sleep(0.2)
        except serial.SerialException:
            pass
        ser.close()
        imu.stop()


if __name__ == "__main__":
    main()
