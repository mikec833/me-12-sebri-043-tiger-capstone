#!/usr/bin/env python3
"""
imu_quaternion_node.py

Converts geometry_msgs/Vector3Stamped roll/pitch/yaw in degrees
(x/y/z, published by imu_node.py on imu/rpy) into a standard sensor_msgs/Imu with the
orientation expressed as a quaternion, republished on imu/data_quat.

Pure debugging/calibration aid: lets you `ros2 topic echo imu/data_quat`
or point rqt_plot / PlotJuggler at it. It does not add any fusion or
correction -- it's the exact same yaw/pitch/roll from imu_node.py, just
re-encoded. Note imu_node.py already publishes yaw in plain degrees on
imu/rpy; that's the more human-readable signal for eyeballing heading
drift. This node exists for cases where you want the standard
sensor_msgs/Imu quaternion format instead (e.g. RViz, robot_localization).

angular_velocity is not provided by the BNO085's UART-RVC frame, and
imu/accel (imu_node.py) is raw, unscaled sensor counts (see
imu_node.py's docstring) -- neither is safe to publish as real
rad/s or m/s^2, so both are left zeroed with covariance[0] = -1
("no estimate"), per the sensor_msgs/Imu convention. Only orientation
is populated.

Published topics
    imu/data_quat    sensor_msgs/Imu
        orientation is the yaw/pitch/roll from imu/rpy converted to a
        quaternion (ZYX intrinsic: yaw about Z, then pitch about Y,
        then roll about X). orientation_covariance is left at all
        zeros ("unknown but usable"), matching the fact that yaw/pitch/
        roll are real, just not yet validated against true north.

Subscribed topics
    imu/rpy    geometry_msgs/Vector3Stamped

CSV export
    On shutdown, writes src/outputs/imu_quat_outputs/imu_quat_log_run<id>.csv
    (one row per message: time_s, stamp_s, qx, qy, qz, qw, plus the
    source yaw/pitch/roll in degrees for easier calibration checks).
    stamp_s is the message's header.stamp as float epoch seconds --
    unlike time_s (seconds since this node started), it's directly
    comparable across nodes/processes, so it's what to join on when
    correlating with imu_log/uwb_log/etc. <id> is the run_id parameter
    -- shared with imu_node.py/uwb_node.py/etc. when started from
    bringup.launch.py, so their CSVs line up. See run_logging.py.

Parameters
    run_id    str    ""
"""

import csv
import math
import os

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from run_logging import resolve_run_id, stamp_to_seconds

# Relative to wherever `ros2 run`/`ros2 launch` is launched from (i.e. ros2_ws/)
OUTPUT_DIR = 'src/outputs/imu_quat_outputs'


def euler_deg_to_quaternion(yaw_deg: float, pitch_deg: float, roll_deg: float):
    """ZYX intrinsic Euler (deg) -> quaternion (x, y, z, w)."""
    yaw = math.radians(yaw_deg) * 0.5
    pitch = math.radians(pitch_deg) * 0.5
    roll = math.radians(roll_deg) * 0.5

    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class ImuQuaternionNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_quaternion_node")

        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter("run_id", "", read_only)

        self._pub = self.create_publisher(Imu, "imu/data_quat", qos_profile_sensor_data)
        self._sub = self.create_subscription(
            Vector3Stamped, "imu/rpy", self._on_imu_rpy, qos_profile_sensor_data
        )

        # For the CSV export: one row per published message
        self.start_time = self.get_clock().now()
        self.log_time = []
        self.log_stamp_s = []
        self.log_qx = []
        self.log_qy = []
        self.log_qz = []
        self.log_qw = []
        self.log_yaw = []
        self.log_pitch = []
        self.log_roll = []

    def _on_imu_rpy(self, rpy: Vector3Stamped) -> None:
        roll, pitch, yaw = rpy.vector.x, rpy.vector.y, rpy.vector.z
        qx, qy, qz, qw = euler_deg_to_quaternion(yaw, pitch, roll)

        msg = Imu()
        msg.header = rpy.header

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance = [0.0] * 9

        # Not available from the RVC frame / not safely convertible -- see docstring.
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self._pub.publish(msg)

        self.log_time.append(self.elapsed_seconds())
        self.log_stamp_s.append(stamp_to_seconds(rpy.header.stamp))
        self.log_qx.append(qx)
        self.log_qy.append(qy)
        self.log_qz.append(qz)
        self.log_qw.append(qw)
        self.log_yaw.append(yaw)
        self.log_pitch.append(pitch)
        self.log_roll.append(roll)

    def elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def save_csv(self) -> None:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        run_id = resolve_run_id(
            OUTPUT_DIR, 'imu_quat_log_run', self.get_parameter('run_id').value
        )
        filename = f'imu_quat_log_run{run_id}.csv'

        with open(os.path.join(OUTPUT_DIR, filename), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                ['time_s', 'stamp_s', 'qx', 'qy', 'qz', 'qw', 'yaw_deg', 'pitch_deg', 'roll_deg']
            )
            writer.writerows(zip(
                self.log_time, self.log_stamp_s, self.log_qx, self.log_qy, self.log_qz,
                self.log_qw, self.log_yaw, self.log_pitch, self.log_roll))

        self.get_logger().info(f'Saved {filename}')

    def destroy_node(self) -> None:
        self.save_csv()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuQuaternionNode()
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
