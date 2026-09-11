#!/usr/bin/env python3
"""
pi_bridge.py
------------------------------------------------------------------------------
Integration/bring-up script for today's Pi + Teensy setup:
  - Talks to the Teensy over USB-serial (Teensy runs
    firmware/teensy_bridge/teensy_bridge.ino, which streams heading and
    encoder ticks and accepts a motor 1 speed command over serial).
  - Leaves a clearly marked slot for the UWB pose reader -- your colleague's
    script goes in get_uwb_pose() below. Everything else is ready for it.
  - Lets you type a speed command to manually drive motor 1 (via a
    Sabertooth 2x12 on its S1 input) while watching heading, encoder ticks,
    and (once wired in) pose update live in the terminal. That's today's
    goal: confirm all of it -- UWB in, IMU heading in, encoder in, motor
    voltage out -- works at the same time.

Only one motor (S1) is wired up right now, so commands are a single
normalized speed in [-1, 1], not a robot-level v/w -- there's no second
motor yet for that to mean anything. See teensy_bridge.ino's header comment
for the plan to bring differential-drive v/w control back once S2 is wired.

Wiring: Teensy connected to the Pi via micro-USB -> USB-A. It should show up
as a serial device -- run `ls /dev/ttyACM*` on the Pi to confirm the exact
name (usually /dev/ttyACM0); adjust SERIAL_PORT below if it differs.
Requires: pip install pyserial

Usage:
    python3 pi_bridge.py
    > 0.2             # drive motor 1 forward at 20% speed
    > -0.2            # drive motor 1 in reverse at 20% speed
    > stop            # stop
    > q               # quit
"""
import re
import sys
import threading
import time

import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200  # Teensy's USB-CDC serial actually ignores this and always runs at full USB
                     # speed -- kept here for clarity / in case you swap to a UART-based board later.

heading_lock = threading.Lock()
latest_heading_rad = None
latest_heading_ts = 0.0

encoder_lock = threading.Lock()
latest_enc_ticks = None
latest_enc_ticks_per_s = None
latest_enc_ts = 0.0


def get_uwb_pose():
    """TODO: wire in your colleague's UWB script here. Return (x, y, z) in
    meters, or None if no fix is available yet. Left as a stub for today --
    everything else in this script (serial link, teleop, terminal display)
    is ready for it to slot straight in; see status_line() below for where
    it gets printed."""
    return None


def serial_reader(ser):
    """Background thread: continuously parses heading and encoder lines from
    the Teensy ('H,<heading_rad>,<millis>' and 'E,<ticks>,<ticks_per_s>,
    <millis>') and stores the latest values. Runs for the life of the
    program so the main thread is free to just handle user input."""
    global latest_heading_rad, latest_heading_ts
    global latest_enc_ticks, latest_enc_ticks_per_s, latest_enc_ts
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except serial.SerialException:
            print("\n# serial read error -- Teensy disconnected?", file=sys.stderr)
            return
        if not line:
            continue
        if line.startswith("H,"):
            parts = line.split(",")
            if len(parts) >= 2:
                try:
                    with heading_lock:
                        latest_heading_rad = float(parts[1])
                        latest_heading_ts = time.monotonic()
                except ValueError:
                    pass
        elif line.startswith("E,"):
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


def send_motor_speed(ser, speed):
    ser.write(f"M,{speed:.4f}\n".encode("ascii"))


def send_stop(ser):
    ser.write(b"S\n")


def status_line():
    with heading_lock:
        h, ts = latest_heading_rad, latest_heading_ts
    if h is None:
        heading_str = "no heading yet"
    else:
        age_ms = (time.monotonic() - ts) * 1000
        heading_str = f"{h:+.3f} rad ({age_ms:.0f} ms old)"

    with encoder_lock:
        ticks, rate = latest_enc_ticks, latest_enc_ticks_per_s
    enc_str = "no encoder data yet" if ticks is None else f"{ticks} ticks ({rate:+.1f} ticks/s)"

    pose = get_uwb_pose()  # <-- pose prints here automatically once get_uwb_pose() is wired in
    pose_str = "pending UWB integration" if pose is None else f"{pose}"

    return f"heading: {heading_str}   |   encoder: {enc_str}   |   pose: {pose_str}"


def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2)
    # Brief pause so the first few reads are stable. Note: unlike an Uno,
    # the Teensy does NOT reboot when a serial connection opens, so this is
    # just settling time, not a reset wait.
    time.sleep(1.0)

    reader = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    reader.start()

    print(f"# connected to {SERIAL_PORT}. Type a speed in [-1, 1] to drive motor 1, 'stop', or 'q' to quit.")
    try:
        while True:
            print(status_line())
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break
            if line in ("stop", "s"):
                send_stop(ser)
                continue
            m = re.match(r"^(-?[\d.]+)$", line)
            if m:
                speed = float(m.group(1))
                if abs(speed) > 1.0:
                    print("# speed should be in [-1, 1] -- sending anyway, but the Teensy will clamp it")
                send_motor_speed(ser, speed)
            else:
                print("# unrecognized -- type a speed (e.g. '0.2' or '-0.2'), 'stop', or 'q'")
    finally:
        send_stop(ser)
        ser.close()


if __name__ == "__main__":
    main()
