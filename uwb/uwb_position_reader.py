"""
uwb_position_reader.py

Raspberry Pi runtime class for reading live position from the
DWM3001CDK tag (DW3_QM33_SDK v1.1.1 CLI firmware, FiRa One-to-Many
mode, 3 anchors). Assumes the tag has already been configured with
`initf -multi -paddr=[...]` and saved to NVM, so it starts ranging
automatically as soon as it's powered.

This module only reads the tag and returns solved (x, y, z)
coordinates. It contains no CSV or file logging; if the caller wants
to log positions, that's their responsibility (e.g. feed them into
the EKF or write them out at a higher level).

Reuses uwb_trilateration.calculate_position() unchanged. That module
holds ANCHOR_POSITIONS and TAG_BELOW_ANCHOR_PLANE as module-level
globals, so UWBPositionReader sets them on the module at construction
time rather than duplicating the trilateration math here.
"""

import re
import time
from typing import Dict, Optional, Tuple

import serial

import uwb_trilateration

# Matches each [mac_address=0x000N, status="...", distance[cm]=N] entry.
# distance[cm] is optional since failed measurements (RX_TIMEOUT,
# TX_FAILED) omit it entirely.
MEASUREMENT_PATTERN = re.compile(
    r'mac_address=0x([0-9A-Fa-f]+),\s*'
    r'status="([A-Z_]+)"'
    r'(?:,\s*distance\[cm\]=(-?\d+))?'
)


class UWBPositionReader:
    """
    Reads ranging blocks from a DWM3001CDK tag over serial and returns
    the trilaterated (x, y, z) position.

    On the ball robot the tag connects to the Pi via micro USB, so
    serial_port will typically be something like "/dev/ttyACM0"
    (confirm with `ls /dev/ttyACM*` while the tag is plugged in; it
    can shift if other USB-serial devices are attached).

    anchor_positions maps anchor_id -> (x, y, z) in the global UWB
    frame, matching the anchor IDs the tag is configured to range
    against (paddr list). This is passed in rather than hardcoded so
    it can be updated after each anchor survey without editing code.
    """

    def __init__(
        self,
        serial_port: str,
        anchor_positions: Dict[int, Tuple[float, float, float]],
        baud_rate: int = 115200,
        tag_below_anchor_plane: bool = True,
        serial_timeout: float = 1.0,
    ):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_timeout = serial_timeout
        self._ser: Optional[serial.Serial] = None

        # uwb_trilateration.calculate_position() reads these as module
        # globals, so configure the module here instead of forking its
        # logic.
        uwb_trilateration.ANCHOR_POSITIONS = dict(anchor_positions)
        uwb_trilateration.TAG_BELOW_ANCHOR_PLANE = tag_below_anchor_plane

    # ---- lifecycle ---------------------------------------------------

    def connect(self) -> None:
        if self._ser is not None and self._ser.is_open:
            return
        self._ser = serial.Serial(
            self.serial_port, self.baud_rate, timeout=self.serial_timeout
        )

    def disconnect(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self) -> "UWBPositionReader":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()

    # ---- serial parsing ------------------------------------------------

    def _read_session_block(self) -> str:
        """
        Read lines starting from a SESSION_INFO_NTF header until the
        braces balance, then return the full notification as one
        string. Handles the notification being split across multiple
        serial lines. Returns "" if the line read wasn't the start of
        a session block (e.g. boot banners, other CLI output).
        """
        if self._ser is None:
            raise RuntimeError("Not connected. Call connect() first.")

        line = self._ser.readline().decode(errors="ignore").strip()
        if not line.startswith("SESSION_INFO_NTF"):
            return ""

        block = line
        depth = block.count("{") - block.count("}")
        while depth > 0:
            next_line = self._ser.readline().decode(errors="ignore").strip()
            if not next_line:
                continue
            block += " " + next_line
            depth += next_line.count("{") - next_line.count("}")

        return block

    @staticmethod
    def _parse_measurements(block: str) -> Dict[int, float]:
        """
        Extract {anchor_id: distance_cm} from a SESSION_INFO_NTF
        block. Only SUCCESS measurements with a valid distance are
        included, so a failed exchange with one anchor just means
        it's missing from the dict for that block.
        """
        ranges: Dict[int, float] = {}
        for mac_hex, status, distance_str in MEASUREMENT_PATTERN.findall(block):
            if status != "SUCCESS" or distance_str == "":
                continue
            anchor_id = int(mac_hex, 16)
            ranges[anchor_id] = float(distance_str)
        return ranges

    # ---- position output ------------------------------------------------

    def read_block(self) -> Optional[dict]:
        """
        Block until one SESSION_INFO_NTF notification is read from the
        tag, then return a dict describing it:

            {
                "timestamp": float,   # time.monotonic() when the block finished reading
                "ranges_m": {anchor_id: distance_m, ...},
                "position": (x, y, z) or None,
                "clamped": bool,      # see uwb_trilateration.calculate_position
            }

        "position" is None whenever fewer than 3 anchors reported a
        successful range in this block, or the anchor geometry can't
        be solved (see uwb_trilateration.calculate_position).

        Returns None if the serial line read wasn't the start of a
        session block (caller should just call again).
        """
        block = self._read_session_block()
        if not block:
            return None

        timestamp = time.monotonic()
        ranges_cm = self._parse_measurements(block)
        ranges_m = {aid: dist / 100.0 for aid, dist in ranges_cm.items()}

        position = None
        clamped = False
        if len(ranges_m) == 3:
            position, clamped = uwb_trilateration.calculate_position(ranges_m)

        return {
            "timestamp": timestamp,
            "ranges_m": ranges_m,
            "position": position,
            "clamped": clamped,
        }

    def get_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Block until a session block with a solvable 3-anchor position
        is read, then return (x, y, z). Skips over blocks with fewer
        than 3 successful ranges or an unsolvable geometry.
        """
        while True:
            result = self.read_block()
            if result is not None and result["position"] is not None:
                return result["position"]

    def stream_positions(self):
        """
        Generator yielding (x, y, z) each time a solvable position
        comes in. Runs indefinitely; the caller controls termination
        (e.g. `for pos in reader.stream_positions(): ...` with a
        `break`, or wrapping the call in a thread that can be
        stopped externally).
        """
        while True:
            yield self.get_position()


if __name__ == "__main__":
    # Minimal usage example. Update these for the deployed hardware.
    anchor_positions = {
        1: (0.0, 0.0, 0.60),
        2: (10.0, 0.0, 0.630),
        3: (0.0, 15.0, 0.610),
    }

    with UWBPositionReader(
        serial_port="/dev/ttyACM0",
        anchor_positions=anchor_positions,
        tag_below_anchor_plane=True,
    ) as reader:
        for x, y, z in reader.stream_positions():
            print(f"x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")
