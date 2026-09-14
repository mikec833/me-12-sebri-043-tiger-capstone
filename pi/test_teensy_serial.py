#!/usr/bin/env python3
"""
test_teensy_serial.py
------------------------------------------------------------------------------
Bench-test tool for the Teensy bridge: prints every heading/encoder line the
Teensy streams as a continuous live feed, while letting you type a motor
speed command at any time without interrupting that feed -- e.g. run this,
watch the encoder ticks sit near zero, type "0.25", and watch the ticks/s
respond live.

Two things this fixes vs. the previous version of this script:
  1. It no longer stops after a fixed 25 reads (~2-5s) -- it now streams
     until you quit.
  2. A typed speed now actually stays in effect. teensy_bridge.ino has a
     500ms command-timeout watchdog (CMD_TIMEOUT_MS) that stops the motor if
     it doesn't hear from us -- sending a command once therefore self-cancels
     in under half a second. This script now runs a background thread that
     keeps re-sending your last commanded speed every 150ms, so it holds
     until you type a new one.

Note: because a background thread is printing sensor lines to the terminal
at the same time you're typing, the prompt has no visible "> " marker (it
would get interleaved with the live feed and look messy) -- just type your
number and press Enter, it'll go through even without a marker to type into.

Usage:
    python3 test_teensy_serial.py
    (heading/encoder lines start streaming immediately)
    0.25      <- type this + Enter: drives motor 1 forward at 25%, holds
    0         <- stop
    q         <- quit
"""
import sys
import threading
import time

import serial

PORTS = [
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]

KEEPALIVE_INTERVAL_S = 0.15  # well under the Teensy's 500ms CMD_TIMEOUT_MS watchdog


def find_port():
    for port in PORTS:
        try:
            s = serial.Serial(port, 115200, timeout=0.2)
            s.close()
            return port
        except Exception:
            pass
    return None


def reader_thread(ser, stop_event):
    """Prints every line the Teensy sends as it arrives -- the continuous
    heading/encoder feed."""
    while not stop_event.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except serial.SerialException:
            print("\n# serial read error -- Teensy disconnected?", file=sys.stderr)
            return
        if line:
            print(line)


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

    stop_event = threading.Event()
    state = {"lock": threading.Lock(), "speed": 0.0}

    threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True).start()
    threading.Thread(target=sender_thread, args=(ser, state, stop_event), daemon=True).start()

    print("# streaming heading/encoder readings below. Type a speed in [-1, 1] "
          "+ Enter to drive motor 1 (it holds until you change it), '0' or "
          "'stop' to stop, 'q' to quit.\n")
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
                print("# unrecognized -- type a speed (e.g. '0.25' or '-0.25'), 'stop', or 'q'")
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
