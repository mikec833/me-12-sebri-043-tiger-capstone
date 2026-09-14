#!/usr/bin/env python3
import serial
import time
import sys

PORTS = [
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]


def find_port():
    for port in PORTS:
        try:
            s = serial.Serial(port, 115200, timeout=0.2)
            s.close()
            return port
        except Exception:
            pass
    return None


if __name__ == "__main__":
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

    print("Sending forward test command: V,0.15,0.0")
    ser.write(b"V,0.15,0.0\n")

    for i in range(25):
        line = ser.readline()
        if line:
            print(line.decode("utf-8", "ignore").strip())
        time.sleep(0.1)

    print("Sending stop command: S")
    ser.write(b"S\n")
    time.sleep(0.5)

    ser.close()
    print("Done.")
