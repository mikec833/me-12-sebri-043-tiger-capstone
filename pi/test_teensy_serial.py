#!/usr/bin/env python3
"""
test_teensy_serial.py
------------------------------------------------------------------------------
Bench-test tool for the Teensy bridge: shows a periodically-updating encoder
summary while letting you type a motor speed command at any time -- e.g. run
this, watch ticks sit near zero, type "0.25", and watch ticks/s respond.

WHY THIS PRINTS A SUMMARY ONCE EVERY PRINT_INTERVAL_S INSTEAD OF EVERY RAW
LINE: an earlier version printed every incoming line immediately (10x/sec,
since the Teensy streams encoder data every 100ms). That's often enough that
a print can land in the middle of you typing a character -- since the
printing thread and your keystrokes share the same terminal, this doesn't
queue politely, it visually SPLICES together, e.g. typing "1" right as a
line prints looks like "1E,0,0.00,39776" -- garbage that's actually your own
keystroke glued to a sensor line. That's almost certainly why some of your
commands got rejected as "unrecognized" -- the interleaving mangled what you
typed before you even pressed Enter. Printing ~3x/second instead of 10x
cuts how often that can happen by roughly the same factor; it won't make it
literally impossible (there's no way to fully guarantee that in a plain
terminal without a full curses-style UI), but it should make it rare enough
to just retype if a line ever looks wrong. Also: avoid the up/down arrow
keys here -- command history recall doesn't work reliably with a background
thread writing to the same terminal, and a stray arrow keypress shows up as
literal "^[[A"-style text (which is itself just another case of the same
interleaving issue).

Two more things this fixes vs. the original version of this script:
  1. It no longer stops after a fixed 25 reads (~2-5s) -- it now runs until
     you quit.
  2. A typed speed now actually stays in effect. teensy_bridge.ino has a
     500ms command-timeout watchdog (CMD_TIMEOUT_MS) that stops the motor if
     it doesn't hear from us -- sending a command once therefore self-cancels
     in under half a second. This script runs a background thread that keeps
     re-sending your last commanded speed every 150ms, so it holds until you
     type a new one.

Also reads heading from the BNO085 over the Pi's own UART (bno085_rvc.py) --
a completely separate physical connection from the Teensy's USB link, so it
runs independently: if the IMU isn't wired up or /dev/serial0 isn't enabled
yet, this script still works for motor+encoder, just shows "no IMU" for
heading instead of crashing.

Also reads position from the UWB tag (uwb/uwb_position_reader.py) -- a
THIRD separate physical connection, its own USB-serial link to the DWM3001CDK
tag (not the Teensy's). Same graceful-degradation pattern as the IMU: if the
tag isn't plugged in, this just shows "not connected" for pose instead of
crashing. ANCHOR_POSITIONS below are placeholders (copied from
uwb_trilateration.py's own defaults) -- update them to your surveyed anchor
positions.

Usage:
    python3 test_teensy_serial.py
    (a "ticks: ... | heading: ... | pose: ... | commanded speed: ..." line
    updates live)
    0.25      <- type this + Enter: drives motor 1 forward at 25%, holds
    0         <- stop
    q         <- quit
"""
import os
import sys
import threading
import time

import serial

from bno085_rvc import BNO085RVCReader

# uwb/ is a sibling directory to pi/, not a package on the default path.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "uwb"))
from uwb_position_reader import UWBPositionReader  # noqa: E402

PORTS = [
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]

KEEPALIVE_INTERVAL_S = 0.15  # well under the Teensy's 500ms CMD_TIMEOUT_MS watchdog
PRINT_INTERVAL_S = 0.3       # how often the summary line updates -- see header comment

# TODO: update to your surveyed anchor positions (meters, global UWB frame) --
# these are just uwb_trilateration.py's own placeholder defaults.
UWB_ANCHOR_POSITIONS = {
    1: (0.0, 0.0, 0.60),
    2: (10.0, 0.0, 0.630),
    3: (0.0, 15.0, 0.610),
}


def find_port(exclude=()):
    for port in PORTS:
        if port in exclude:
            continue
        try:
            s = serial.Serial(port, 115200, timeout=0.2)
            s.close()
            return port
        except Exception:
            pass
    return None


def reader_thread(ser, state, stop_event):
    """Parses encoder lines from the Teensy ('E,<ticks>,<ticks_per_s>,
    <millis>') into shared state -- does NOT print directly; see
    printer_thread for why that's split out separately."""
    while not stop_event.is_set():
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
                    with state["lock"]:
                        state["ticks"] = int(parts[1])
                        state["ticks_per_s"] = float(parts[2])
                except ValueError:
                    pass
        elif line.startswith("#"):
            print(line)  # rare, low-frequency -- fine to print immediately


def uwb_thread(reader, state, stop_event):
    """UWBPositionReader.read_block() is blocking (its own readline() with a
    1s timeout), so this runs it in a loop on its own thread the same way
    reader_thread does for the Teensy -- updates shared state, doesn't print
    directly."""
    while not stop_event.is_set():
        try:
            result = reader.read_block()
        except serial.SerialException:
            print("\n# serial read error -- UWB tag disconnected?", file=sys.stderr)
            return
        if result is None:
            continue  # not the start of a session block (boot banner, etc) -- try again
        with state["lock"]:
            state["uwb_position"] = result["position"]
            state["uwb_anchor_count"] = len(result["ranges_m"])


def printer_thread(state, imu, has_uwb, stop_event):
    """Prints one summary line every PRINT_INTERVAL_S instead of printing
    every raw serial line -- see header comment for why."""
    while not stop_event.is_set():
        with state["lock"]:
            ticks, rate, speed = state["ticks"], state["ticks_per_s"], state["speed"]
            uwb_position, uwb_anchor_count = state["uwb_position"], state["uwb_anchor_count"]

        if imu is None:
            heading_str = "not connected"
        else:
            yaw, _pitch, _roll, age_s = imu.latest()
            heading_str = "no data yet" if yaw is None else f"{yaw:+.2f} deg ({age_s * 1000:.0f}ms old)"

        if not has_uwb:
            pose_str = "not connected"
        elif uwb_position is None:
            pose_str = f"no fix yet ({uwb_anchor_count}/3 anchors)"
        else:
            x, y, z = uwb_position
            pose_str = f"x={x:+.2f} y={y:+.2f} z={z:+.2f} m"

        print(f"ticks: {ticks:6d}  ({rate:+6.1f} ticks/s)   |   heading: {heading_str}   |   "
              f"pose: {pose_str}   |   commanded speed: {speed:+.2f}")
        time.sleep(PRINT_INTERVAL_S)


def sender_thread(ser, state, stop_event):
    """Keeps re-sending the currently commanded speed so it doesn't get cut
    off by the Teensy's 500ms command-timeout watchdog."""
    while not stop_event.is_set():
        with state["lock"]:
            speed = state["speed"]
        try:
            ser.write(f"M,{speed:.4f}\n".encode("ascii"))
        except serial.SerialException:
            return
        time.sleep(KEEPALIVE_INTERVAL_S)


def main():
    port = find_port()
    if port is None:
        print("No Teensy serial port found.")
        print("Tried:")
        for p in PORTS:
            print("  -", p)
        print("\nCheck that the Teensy is connected and powered.")
        sys.exit(1)

    print(f"Using serial port: {port}")
    ser = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(1.0)

    try:
        imu = BNO085RVCReader().start()
    except Exception as e:
        print(f"# could not open IMU UART ({e}) -- continuing without heading")
        imu = None

    uwb_port = find_port(exclude={port})
    uwb_reader = None
    if uwb_port is None:
        print("# no UWB serial port found -- continuing without pose")
    else:
        try:
            uwb_reader = UWBPositionReader(uwb_port, UWB_ANCHOR_POSITIONS)
            uwb_reader.connect()
            print(f"# UWB tag on {uwb_port}")
        except Exception as e:
            print(f"# could not open UWB serial ({e}) -- continuing without pose")
            uwb_reader = None

    stop_event = threading.Event()
    state = {
        "lock": threading.Lock(),
        "speed": 0.0,
        "ticks": 0,
        "ticks_per_s": 0.0,
        "uwb_position": None,
        "uwb_anchor_count": 0,
    }

    threading.Thread(target=reader_thread, args=(ser, state, stop_event), daemon=True).start()
    threading.Thread(target=printer_thread, args=(state, imu, uwb_reader is not None, stop_event), daemon=True).start()
    threading.Thread(target=sender_thread, args=(ser, state, stop_event), daemon=True).start()
    if uwb_reader is not None:
        threading.Thread(target=uwb_thread, args=(uwb_reader, state, stop_event), daemon=True).start()

    print("# type a speed in [-1, 1] + Enter to drive motor 1 (it holds until you "
          "change it), '0' or 'stop' to stop, 'q' to quit. Avoid arrow keys (see "
          "header comment). If a line ever looks garbled, just retype it.\n")
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
                with state["lock"]:
                    state["speed"] = 0.0
                continue
            try:
                speed = float(line)
            except ValueError:
                print("# unrecognized -- type a speed (e.g. '0.25' or '-0.25'), 'stop', or 'q'. "
                      "If you're sure you typed a valid number, this may be interleaving "
                      "garbage (see header comment) -- just retype it.")
                continue
            if abs(speed) > 1.0:
                print("# speed should be in [-1, 1] -- sending anyway, the Teensy will clamp it")
            with state["lock"]:
                state["speed"] = speed
    finally:
        stop_event.set()
        try:
            ser.write(b"S\n")
            time.sleep(0.2)
        except serial.SerialException:
            pass
        ser.close()
        if imu is not None:
            imu.stop()
        if uwb_reader is not None:
            uwb_reader.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
