#!/usr/bin/env python3
"""
uwb_node.py

ROS 2 (rclpy) publisher node for the DWM3001CDK UWB tag. Wraps
UWBPositionReader (unchanged) and publishes the trilaterated tag
position. No filtering, no covariance; the EKF owns those.

Published topics
    uwb/position  geometry_msgs/PointStamped
        Trilaterated (x, y, z) in metres in frame_id. Published only
        when a 3-anchor solution exists.

Timestamp
    header.stamp is the time the sample was taken. The tag's
    SESSION_INFO_NTF output carries no timestamp, so this is estimated
    on the host: the moment the block finished arriving over serial
    (time.monotonic() inside the reader, converted to the ROS clock),
    minus latency_compensation_s. It is never the publish time.

Parameters (read only, set at startup)
    serial_port              str          "/dev/ttyACM0"
    baud_rate                int          115200
    serial_timeout_s         float        1.0
    reconnect_period_s       float        2.0
    anchor_ids               int[3]       [1, 2, 3]
    anchor_positions         float[9]     x1,y1,z1,x2,y2,z2,x3,y3,z3 (m)
    tag_below_anchor_plane   bool         True

Parameters (dynamic)
    frame_id                 str          "map"
    latency_compensation_s   float        0.0
        Fixed delay between the ranging measurement and the block
        arriving at the Pi (ranging round, tag CLI output, USB
        transfer). Subtracted from the receipt time. Measure it before
        relying on a non-zero value.
"""

import csv
import glob
import os
import threading
import time
import traceback
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from uwb_position_reader import UWBPositionReader

# Relative to wherever `ros2 run`/`ros2 launch` is launched from (i.e. ros2_ws/)
OUTPUT_DIR = 'src/outputs/uwb_outputs'


class UwbNode(Node):
    def __init__(self) -> None:
        super().__init__("uwb_node")

        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter("serial_port", "/dev/ttyACM0", read_only)
        self.declare_parameter("baud_rate", 115200, read_only)
        self.declare_parameter("serial_timeout_s", 1.0, read_only)
        self.declare_parameter("reconnect_period_s", 2.0, read_only)
        self.declare_parameter("anchor_ids", [1, 2, 3], read_only)
        self.declare_parameter(
            "anchor_positions",
            [0.0, 0.0, 1.3, 1.8, 0.0, 1.3, 0.0, 1.8, 1.3],
            read_only,
        )
        self.declare_parameter("tag_below_anchor_plane", True, read_only)

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("latency_compensation_s", 0.0)

        anchor_positions = self._load_anchors()
        self._serial_timeout_s = float(self.get_parameter("serial_timeout_s").value)
        self._reconnect_period_s = float(self.get_parameter("reconnect_period_s").value)

        self._reader = UWBPositionReader(
            serial_port=self.get_parameter("serial_port").value,
            anchor_positions=anchor_positions,
            baud_rate=int(self.get_parameter("baud_rate").value),
            tag_below_anchor_plane=bool(self.get_parameter("tag_below_anchor_plane").value),
            serial_timeout=self._serial_timeout_s,
        )

        self._pub_position = self.create_publisher(
            PointStamped, "uwb/position", qos_profile_sensor_data
        )

        # For the CSV export: one row per published position
        self.start_time = self.get_clock().now()
        self.log_time = []
        self.log_x = []
        self.log_y = []
        self.log_z = []
        self.log_clamped = []

        # The serial read blocks, so it runs in its own thread instead of
        # an executor callback. rclpy publishers are safe to call from
        # this thread.
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._reader_loop, name="uwb_reader", daemon=True
        )
        self._thread.start()
        self.get_logger().info(
            f"UWB node started on {self.get_parameter('serial_port').value}"
        )

    # ---- configuration -----------------------------------------------

    def _load_anchors(self) -> Dict[int, Tuple[float, float, float]]:
        ids = [int(a) for a in self.get_parameter("anchor_ids").value]
        flat = [float(v) for v in self.get_parameter("anchor_positions").value]
        if len(ids) != 3 or len(set(ids)) != 3:
            raise ValueError(f"anchor_ids must hold exactly 3 unique IDs, got {ids}")
        if len(flat) != 9:
            raise ValueError(
                f"anchor_positions must hold 9 values (x,y,z per anchor), got {len(flat)}"
            )
        return {
            aid: (flat[3 * k], flat[3 * k + 1], flat[3 * k + 2])
            for k, aid in enumerate(ids)
        }

    # ---- reader thread ---------------------------------------------------

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                self._reader.connect()
                result = self._reader.read_block()
            except Exception as exc:  # serial faults, unplugged tag, parse errors
                if self._stop_event.is_set():
                    break
                self.get_logger().warn(
                    f"UWB link fault ({type(exc).__name__}: {exc}), "
                    f"reconnecting in {self._reconnect_period_s:.1f} s",
                    throttle_duration_sec=5.0,
                )
                self.get_logger().debug(traceback.format_exc())
                self._safe_disconnect()
                self._stop_event.wait(self._reconnect_period_s)
                continue

            if result is None:
                # Line was not a SESSION_INFO_NTF header (or the read timed out).
                continue

            self._publish_position(result)

    def _publish_position(self, result: dict) -> None:
        position = result["position"]
        if position is None:
            self.get_logger().warn(
                f"No position solution: {len(result['ranges_m'])}/3 anchors returned a range",
                throttle_duration_sec=5.0,
            )
            return
        if result["clamped"]:
            self.get_logger().warn(
                "Clamped solution (ranges inconsistent with anchor geometry)",
                throttle_duration_sec=5.0,
            )

        # Sample time: reader's receipt time (monotonic clock) minus the
        # configured measurement-to-receipt latency, mapped onto the ROS clock.
        latency_s = float(self.get_parameter("latency_compensation_s").value)
        age_s = max(0.0, time.monotonic() - result["timestamp"] + latency_s)
        stamp = (self.get_clock().now() - Duration(seconds=age_s)).to_msg()

        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.point.x = float(position[0])
        msg.point.y = float(position[1])
        msg.point.z = float(position[2])
        self._pub_position.publish(msg)

        self.log_time.append(self.elapsed_seconds())
        self.log_x.append(msg.point.x)
        self.log_y.append(msg.point.y)
        self.log_z.append(msg.point.z)
        self.log_clamped.append(result["clamped"])

    def elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    # ---- lifecycle ---------------------------------------------------------

    def _safe_disconnect(self) -> None:
        try:
            self._reader.disconnect()
        except Exception:
            pass

    def next_run_number(self) -> int:
        existing = glob.glob(os.path.join(OUTPUT_DIR, 'uwb_log_run*.csv'))
        run_numbers = [0]
        for path in existing:
            digits = os.path.basename(path)[len('uwb_log_run'):-len('.csv')]
            if digits.isdigit():
                run_numbers.append(int(digits))
        return max(run_numbers) + 1

    def save_csv(self) -> None:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        filename = f'uwb_log_run{self.next_run_number()}.csv'

        with open(os.path.join(OUTPUT_DIR, filename), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'x', 'y', 'z', 'clamped'])
            writer.writerows(zip(
                self.log_time, self.log_x, self.log_y, self.log_z, self.log_clamped))

        self.get_logger().info(f'Saved {filename}')

    def destroy_node(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=self._serial_timeout_s + 1.0)
        self._safe_disconnect()
        self.save_csv()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UwbNode()
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
