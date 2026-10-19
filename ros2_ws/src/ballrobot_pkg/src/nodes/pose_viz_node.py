#!/usr/bin/env python3
"""
pose_viz_node.py

Combines the live UWB position and IMU yaw into one PoseStamped, so the
Foxglove 3D panel can show the ball robot's location and heading as a
single arrow on the top-down XY plane. Visualization only; nothing else
on the robot reads viz/pose.

Published topics
    viz/pose      geometry_msgs/PoseStamped
        position.x/y from uwb/position (position.z = 0), orientation a
        yaw-only quaternion from the latest imu/rpy yaw. Published on
        every UWB message once at least one IMU message has arrived.
        header.stamp is the UWB message's stamp.

Subscribed topics
    uwb/position  geometry_msgs/PointStamped      (uwb_node.py, metres)
    imu/rpy       geometry_msgs/Vector3Stamped    (imu_node.py, degrees;
                                                   z = yaw)

Heading
    yaw = wrap_[-pi, pi)(radians(yaw_sign * imu_yaw_deg + yaw_offset_deg))
    q   = (0, 0, sin(yaw / 2), cos(yaw / 2))
    Roll and pitch are ignored.

Parameters (read only, set at startup)
    frame_id         str    "map"
    imu_timeout_s    float  0.5
        If the last IMU stamp is older than this when a UWB message
        arrives, the pose is still published with the last yaw and a
        throttled warning is logged.

Parameters (dynamic, re-read on every IMU message)
    yaw_sign         float  1.0    +1 or -1
    yaw_offset_deg   float  0.0
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle in radians to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PoseVizNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_viz_node')

        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter('frame_id', 'map', read_only)
        self.declare_parameter('imu_timeout_s', 0.5, read_only)

        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('yaw_offset_deg', 0.0)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._imu_timeout_s = float(self.get_parameter('imu_timeout_s').value)

        # Latest heading (rad) and the stamp of the IMU message it came from;
        # None until the first imu/rpy message arrives.
        self._yaw = None
        self._imu_stamp = None

        self._pub_pose = self.create_publisher(
            PoseStamped, 'viz/pose', qos_profile_sensor_data
        )
        self.create_subscription(
            Vector3Stamped, 'imu/rpy', self._on_imu_rpy, qos_profile_sensor_data
        )
        self.create_subscription(
            PointStamped, 'uwb/position', self._on_uwb_position, qos_profile_sensor_data
        )

        self.get_logger().info(f'Pose viz node started, publishing viz/pose in {self._frame_id}')

    def _on_imu_rpy(self, rpy: Vector3Stamped) -> None:
        # Read every time so `ros2 param set` takes effect live
        yaw_sign = float(self.get_parameter('yaw_sign').value)
        yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').value)
        self._yaw = wrap_to_pi(math.radians(yaw_sign * rpy.vector.z + yaw_offset_deg))
        self._imu_stamp = Time.from_msg(rpy.header.stamp)

    def _on_uwb_position(self, uwb: PointStamped) -> None:
        if self._yaw is None:
            return

        imu_age_s = (Time.from_msg(uwb.header.stamp) - self._imu_stamp).nanoseconds * 1e-9
        if imu_age_s > self._imu_timeout_s:
            self.get_logger().warn(
                f'imu/rpy is stale ({imu_age_s:.2f} s older than uwb/position), '
                'publishing the last heading',
                throttle_duration_sec=5.0,
            )

        msg = PoseStamped()
        msg.header.stamp = uwb.header.stamp
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = uwb.point.x
        msg.pose.position.y = uwb.point.y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(self._yaw * 0.5)
        msg.pose.orientation.w = math.cos(self._yaw * 0.5)
        self._pub_pose.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
