#!/usr/bin/env python3
"""
test_teensy_serial.py  (laptop version)
------------------------------------------------------------------------------
Same bench-test tool as pi/test_teensy_serial.py -- send a motor speed
command, watch encoder ticks respond -- but for testing the Teensy directly
from your laptop over USB (bypassing the Pi entirely), e.g. to isolate the
Teensy/Sabertooth/encoder hardware from anything Pi-side.

The only real difference from the Pi version is port auto-detection: on
macOS, a Teensy enumerates as /dev/cu.usbmodemXXXXXXX (the exact number
varies per connect/board), not /dev/ttyACM0 like on Linux. This script globs
for that pattern first, then falls back to the Linux-style names in case you
ever run it from a Linux machine instead.

WHY THIS PRINTS A SUMMARY ONCE EVERY PRINT_INTERVAL_S INSTEAD OF EVERY RAW
LINE: printing every incoming line immediately (10x/sec, since the Teensy
streams encoder data every 100ms) risks a print landing mid-keystroke --
since the printing thread and your typing share the same terminal, that
doesn't queue politely, it visually splices together (e.g. typing "1" right
as a line prints looks like "1E,0,0.00,39776" -- garbage that's actually
your own keystroke glued to a sensor line, and a likely cause of commands
getting rejected as "unrecognized"). Printing ~3x/second instead of 10x
cuts how often that can happen; it won't make it literally impossible
without a full curses-style UI, but should make it rare enough that a "just
retype it" fallback is practical. Also avoid the up/down arrow keys here --
command history recall doesn't work reliably with a background thread
writing to the same terminal.

Usage:
    python3 test_teensy_serial.py
    (a "ticks: ... | commanded speed: ..." line starts updating immediately)
    0.25      <- type this + Enter: drives motor 1 forward at 25%, holds
    0         <- stop
    q         <- quit
"""
import glob
import sys
import threading
import time

import serial

# Checked in this order; first match wins.
MACOS_GLOB = "/dev/cu.usbmodem*"
FALLBACK_PORTS = [
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]

KEEPALIVE_INTERVAL_S = 0.15  # well under the Teensy's 500ms CMD_TIMEOUT_MS watchdog
PRINT_INTERVAL_S = 0.3       # how often the summary line updates -- see header comment


def find_port():
    matches = sorted(glob.glob(MACOS_GLOB))
    if matches:
        if len(matches) > 1:
            print(f"# multiple matches for {MACOS_GLOB}, using the first: {matches}")
        return matches[0]
    for port in FALLBACK_PORTS:
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


def printer_thread(state, stop_event):
    """Prints one summary line every PRINT_INTERVAL_S instead of printing
    every raw serial line -- see header comment for why."""
    while not stop_event.is_set():
        with state["lock"]:
            ticks, rate, speed = state["ticks"], state["ticks_per_s"], state["speed"]
        print(f"ticks: {ticks:6d}  ({rate:+6.1f} ticks/s)   |   commanded speed: {speed:+.2f}")
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
        print(f"No Teensy serial port found (looked for {MACOS_GLOB} and {FALLBACK_PORTS}).")
        print("Check that the Teensy is connected via USB and powered.")
        sys.exit(1)

    print(f"Using serial port: {port}")
    ser = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(1.0)

    stop_event = threading.Event()
    state = {"lock": threading.Lock(), "speed": 0.0, "ticks": 0, "ticks_per_s": 0.0}

    threading.Thread(target=reader_thread, args=(ser, state, stop_event), daemon=True).start()
    threading.Thread(target=printer_thread, args=(state, stop_event), daemon=True).start()
    threading.Thread(target=sender_thread, args=(ser, state, stop_event), daemon=True).start()

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
        print("Done.")


if __name__ == "__main__":
    main()
