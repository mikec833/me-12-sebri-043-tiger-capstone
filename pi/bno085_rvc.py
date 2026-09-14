#!/usr/bin/env python3
"""
bno085_rvc.py
------------------------------------------------------------------------------
Reads a BNO085 wired directly to the Pi's UART (GPIO14/15, exposed as
/dev/serial0 on Raspberry Pi OS) in UART-RVC ("simplified") mode. This mode
needs no configuration or handshake -- once the chip's PS0/PS1 pins are
strapped for it, it just continuously streams a fixed-format frame at
115200 baud on its own. (The BNO085's SCL/SDA pins double as RX/TX in this
mode -- that's why wiring described as "SCL/SDA to RX/TX" makes sense here.)

If it turns out your board is actually in full UART-SHTP mode instead (same
protocol as I2C/SPI, just framed over UART -- needs a report-enable
handshake, not just continuous streaming), this file's frame parser doesn't
apply and would need to be swapped for an SHTP-over-UART implementation.
Telling them apart is easy in practice: RVC mode produces valid, continuously
streaming frames the instant you open the port with no setup; if you open
this and get nothing/garbage no matter what, that's a sign it might be SHTP
mode instead (or the PS0/PS1 straps/wiring need checking).

RASPBERRY PI SETUP NEEDED FIRST (common gotcha): the primary GPIO UART is,
by default, often used for the Linux login console and/or shared internally
with Bluetooth depending on Pi model. Run:
    sudo raspi-config -> Interface Options -> Serial Port
    -> "login shell over serial?"  NO
    -> "enable serial port hardware?"  YES
then reboot. Without this, /dev/serial0 either won't exist, will be the
console (fighting with this script), or will be the wrong UART.

FRAME FORMAT (matches the commonly documented BNO08x UART-RVC layout -- if
readings look wrong or checksums keep failing, this is the first thing to
re-verify against your module's actual datasheet):
    byte  0    : 0xAA (sync)
    byte  1    : 0xAA (sync)
    byte  2    : frame index (rolls over 0-255)
    bytes 3-4  : yaw   (int16, LSB first, 0.01 deg/LSB)
    bytes 5-6  : pitch (int16, LSB first, 0.01 deg/LSB)
    bytes 7-8  : roll  (int16, LSB first, 0.01 deg/LSB)
    bytes 9-14 : x/y/z acceleration (int16 each, LSB first) -- unused here
    bytes 15-17: reserved
    byte  18   : checksum = sum(bytes 2..17) & 0xFF

Run this file directly for a standalone smoke test (just prints live
yaw/pitch/roll); import BNO085RVCReader to use it from another script (see
pi_bridge.py for an example).
"""
import struct
import sys
import threading
import time

import serial

DEFAULT_PORT = "/dev/serial0"
BAUD_RATE = 115200
FRAME_LEN = 19
SYNC = b"\xAA\xAA"


class BNO085RVCReader:
    """Background-threaded UART-RVC reader. Call start(), then latest() and
    health() from any thread at any time."""

    def __init__(self, port=DEFAULT_PORT, baud=BAUD_RATE):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self._lock = threading.Lock()
        self._yaw_deg = None
        self._pitch_deg = None
        self._roll_deg = None
        self._ts = 0.0
        self._stop = threading.Event()
        self._thread = None
        self._frame_count = 0
        self._checksum_fail_count = 0

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self.ser.close()

    def _run(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(64)
            except serial.SerialException:
                return
            if not chunk:
                continue
            buf += chunk
            while True:
                idx = buf.find(SYNC)
                if idx == -1:
                    if len(buf) > 1:
                        del buf[:-1]  # keep last byte in case it's half a sync pattern
                    break
                if idx > 0:
                    del buf[:idx]  # drop garbage before the sync bytes
                if len(buf) < FRAME_LEN:
                    break  # wait for the rest of the frame to arrive
                frame = bytes(buf[:FRAME_LEN])
                del buf[:FRAME_LEN]
                self._handle_frame(frame)

    def _handle_frame(self, frame):
        self._frame_count += 1
        checksum = sum(frame[2:18]) & 0xFF
        if checksum != frame[18]:
            self._checksum_fail_count += 1
            return  # drop it rather than report a garbage reading
        yaw_raw, pitch_raw, roll_raw = struct.unpack_from("<hhh", frame, 3)
        with self._lock:
            self._yaw_deg = yaw_raw / 100.0
            self._pitch_deg = pitch_raw / 100.0
            self._roll_deg = roll_raw / 100.0
            self._ts = time.monotonic()

    def latest(self):
        """(yaw_deg, pitch_deg, roll_deg, age_s), or all-None if no valid
        frame has been received yet."""
        with self._lock:
            if self._yaw_deg is None:
                return None, None, None, None
            return self._yaw_deg, self._pitch_deg, self._roll_deg, time.monotonic() - self._ts

    def health(self):
        """(frame_count, checksum_fail_count). A high failure rate relative
        to frame_count means the format/baud/wiring assumptions above are
        probably wrong for your specific module."""
        return self._frame_count, self._checksum_fail_count


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    print(f"# reading BNO085 UART-RVC on {port} -- Ctrl-C to stop")
    reader = BNO085RVCReader(port=port).start()
    try:
        while True:
            yaw, pitch, roll, age = reader.latest()
            frames, fails = reader.health()
            if yaw is None:
                print(f"# no valid frames yet (seen {frames}, {fails} checksum failures) "
                      f"-- check wiring/PS0-PS1 mode if this doesn't change")
            else:
                print(f"yaw={yaw:+7.2f} deg  pitch={pitch:+7.2f}  roll={roll:+7.2f}  "
                      f"age={age * 1000:.0f}ms  (frames={frames}, checksum failures={fails})")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
