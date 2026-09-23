#!/usr/bin/env python3
"""
imu_node.py

ROS 2 (rclpy) publisher node for the BNO085 in UART-RVC mode, wired
directly to the Pi's UART. No filtering, no quaternion conversion; the
EKF owns fusion.

This node does not import BNO085RVCReader from bno085_rvc.py. That
class caches the latest frame for polling (latest()/health()), with no
"new data" signal, which fits a timer-driven consumer, not a
publish-as-received one. Instead this node re-implements the same
frame sync, unpack and checksum logic (see bno085_rvc.py for the frame
layout and the RVC-vs-SHTP mode notes) directly inside its own reader
thread, and publishes the instant a frame's checksum passes.
bno085_rvc.py itself is unmodified; run it standalone if you want the
plain non-ROS smoke test it already provides.

Published topics
    imu/data    ballrobot_pkg/ImuRvc
        One message per valid frame, carrying yaw/pitch/roll (deg) and
        accel_x/y/z (raw signed 16-bit counts, UNCONVERTED) together,
        since they're decoded from the same 19-byte frame. The source
        frame format documents yaw/pitch/roll as 0.01 deg/LSB but does
        not document an accel scale ("x/y/z acceleration (int16 each,
        LSB first) -- unused here"). Treat accel_x/y/z as raw sensor
        counts, not m/s^2 or g, until the LSB scale is confirmed
        against CEVA's BNO08x datasheet. See
        msg/ImuRvc.msg (in this package) for field definitions.

Publishes only when a frame arrives with a valid checksum. Nothing is
published before the first valid frame, and nothing is published for
a frame that fails its checksum.

Timestamp
    header.stamp is the time the sample was taken, not the publish
    time: the moment the frame's checksum passed on the reader thread
    (time.monotonic()), converted to the ROS clock, minus
    latency_compensation_s. Same convention as uwb_node.py.

Parameters (read only, set at startup)
    serial_port              str     "/dev/serial0"
    baud_rate                int     115200
    serial_read_chunk        int     64      (bytes per ser.read() call)
    reconnect_period_s       float   2.0

Parameters (dynamic)
    frame_id                  str     "imu_link"
    latency_compensation_s    float   0.0
        Fixed delay between the physical sample and the frame's
        checksum byte finishing arrival on the UART. Subtracted from
        the stamp. Leave at 0.0 until measured.
"""

import struct
import threading
import time
import traceback

import rclpy
import serial
from ballrobot_pkg.msg import ImuRvc
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

FRAME_LEN = 19
SYNC = b"\xAA\xAA"


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_node")

        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter("serial_port", "/dev/serial0", read_only)
        self.declare_parameter("baud_rate", 115200, read_only)
        self.declare_parameter("serial_read_chunk", 64, read_only)
        self.declare_parameter("reconnect_period_s", 2.0, read_only)

        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("latency_compensation_s", 0.0)

        self._serial_port = self.get_parameter("serial_port").value
        self._baud_rate = int(self.get_parameter("baud_rate").value)
        self._read_chunk = int(self.get_parameter("serial_read_chunk").value)
        self._reconnect_period_s = float(self.get_parameter("reconnect_period_s").value)

        self._ser = None
        self._frame_count = 0
        self._checksum_fail_count = 0

        self._pub_imu = self.create_publisher(
            ImuRvc, "imu/data", qos_profile_sensor_data
        )

        # Blocking serial reads, so this runs in its own thread rather
        # than an executor callback, same as uwb_node.py.
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._reader_loop, name="imu_reader", daemon=True
        )
        self._thread.start()
        self.get_logger().info(f"IMU node started on {self._serial_port}")

    # ---- serial lifecycle -----------------------------------------------

    def _connect(self) -> None:
        if self._ser is not None and self._ser.is_open:
            return
        self._ser = serial.Serial(self._serial_port, self._baud_rate, timeout=0.2)

    def _safe_disconnect(self) -> None:
        try:
            if self._ser is not None and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    # ---- reader thread ------------------------------------------------------

    def _reader_loop(self) -> None:
        buf = bytearray()
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                self._connect()
                chunk = self._ser.read(self._read_chunk)
            except Exception as exc:  # serial faults, unplugged UART adapter
                if self._stop_event.is_set():
                    break
                self.get_logger().warn(
                    f"IMU link fault ({type(exc).__name__}: {exc}), "
                    f"reconnecting in {self._reconnect_period_s:.1f} s",
                    throttle_duration_sec=5.0,
                )
                self.get_logger().debug(traceback.format_exc())
                self._safe_disconnect()
                self._stop_event.wait(self._reconnect_period_s)
                continue

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
                # Timestamp the instant the full, still-unverified frame
                # finished arriving, before checksum/publish work.
                self._handle_frame(frame, time.monotonic())

    def _handle_frame(self, frame: bytes, receipt_time: float) -> None:
        self._frame_count += 1
        checksum = sum(frame[2:18]) & 0xFF
        if checksum != frame[18]:
            self._checksum_fail_count += 1
            if self._checksum_fail_count % 50 == 1:
                self.get_logger().warn(
                    f"IMU checksum failures: {self._checksum_fail_count}/"
                    f"{self._frame_count} frames "
                    "(check baud/wiring/PS0-PS1 mode if this keeps climbing)",
                    throttle_duration_sec=5.0,
                )
            return  # drop it rather than publish a garbage reading

        yaw_raw, pitch_raw, roll_raw = struct.unpack_from("<hhh", frame, 3)
        accel_x_raw, accel_y_raw, accel_z_raw = struct.unpack_from("<hhh", frame, 9)

        latency_s = float(self.get_parameter("latency_compensation_s").value)
        age_s = max(0.0, time.monotonic() - receipt_time + latency_s)
        stamp = (self.get_clock().now() - Duration(seconds=age_s)).to_msg()

        msg = ImuRvc()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.yaw = yaw_raw / 100.0
        msg.pitch = pitch_raw / 100.0
        msg.roll = roll_raw / 100.0
        msg.accel_x = float(accel_x_raw)
        msg.accel_y = float(accel_y_raw)
        msg.accel_z = float(accel_z_raw)
        self._pub_imu.publish(msg)

    # ---- lifecycle -----------------------------------------------------------

    def destroy_node(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._safe_disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
