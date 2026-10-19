#!/usr/bin/env python3
"""
pose_viz_node.py

Combines the live UWB position and IMU yaw into one PoseStamped, so the
Foxglove 3D panel can show the ball robot's location and heading as a
single arrow on the top-down XY plane. Visualization only; nothing else
on the robot reads viz/pose.

Published topics
    viz/pose      geometry_msgs/PoseStamped
        position.x/y from uwb/position, smoothed (see Position
        smoothing; position.z = 0), orientation a
        yaw-only quaternion from the latest imu/rpy yaw. Published on
        every UWB message once at least one IMU message has arrived.
        header.stamp is the UWB message's stamp.

Subscribed topics
    uwb/position  geometry_msgs/PointStamped      (uwb_node.py, metres)
    imu/rpy       geometry_msgs/Vector3Stamped    (imu_node.py, degrees;
                                                   z = yaw)
    viz/set_heading std_msgs/Float64              degrees in frame_id
        Re-aligns the heading: "the robot is facing this way right now".
        Sets yaw_offset_deg so the current yaw equals the given heading
        (0 = +x, 90 = +y), cancelling the IMU's drift up to that moment.
        Publish it from a Foxglove Publish panel, or e.g.
            ros2 topic pub --once /viz/set_heading std_msgs/msg/Float64 '{data: 0.0}'

Heading
    yaw = wrap_[-pi, pi)(radians(yaw_sign * imu_yaw_deg + yaw_offset_deg))
    q   = (0, 0, sin(yaw / 2), cos(yaw / 2))
    Roll and pitch are ignored.

Position smoothing
    Exponential moving average on x/y, for display only (uwb/position
    itself is untouched, so feeder_trigger_node and the bag see raw fixes):
        p = alpha * p_uwb + (1 - alpha) * p
    alpha = 1.0 turns it off; lower is smoother but lags more. At the
    tag's ~5 Hz, 0.3 settles in roughly a second.

Parameters (read only, set at startup)
    frame_id         str    "map"
    imu_timeout_s    float  0.5
        If the last IMU stamp is older than this when a UWB message
        arrives, the pose is still published with the last yaw and a
        throttled warning is logged.

Parameters (dynamic, re-read on every IMU message)
    yaw_sign         float  1.0    +1 or -1
    yaw_offset_deg   float  0.0    also set by viz/set_heading

Parameters (dynamic, re-read on every UWB message)
    position_alpha   float  0.3    EMA weight of the newest fix, (0, 1]
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float64


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle in radians to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def wrap_to_180(angle_deg: float) -> float:
    """Wrap an angle in degrees to [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


class PoseVizNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_viz_node')

        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter('frame_id', 'map', read_only)
        self.declare_parameter('imu_timeout_s', 0.5, read_only)

        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('position_alpha', 0.3)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._imu_timeout_s = float(self.get_parameter('imu_timeout_s').value)

        # Latest heading (rad) and the stamp of the IMU message it came from;
        # None until the first imu/rpy message arrives.
        self._yaw = None
        self._imu_stamp = None
        # Raw imu/rpy yaw (deg) from the latest message, for viz/set_heading
        self._imu_yaw_deg = None
        # Smoothed (x, y); None until the first UWB fix seeds it.
        self._xy = None

        self._pub_pose = self.create_publisher(
            PoseStamped, 'viz/pose', qos_profile_sensor_data
        )
        self.create_subscription(
            Vector3Stamped, 'imu/rpy', self._on_imu_rpy, qos_profile_sensor_data
        )
        self.create_subscription(
            PointStamped, 'uwb/position', self._on_uwb_position, qos_profile_sensor_data
        )
        self.create_subscription(Float64, 'viz/set_heading', self._on_set_heading, 10)

        self.get_logger().info(f'Pose viz node started, publishing viz/pose in {self._frame_id}')

    def _on_imu_rpy(self, rpy: Vector3Stamped) -> None:
        self._imu_yaw_deg = rpy.vector.z
        self._imu_stamp = Time.from_msg(rpy.header.stamp)
        self._update_yaw()

    def _update_yaw(self) -> None:
        # Read every time so `ros2 param set` takes effect live
        yaw_sign = float(self.get_parameter('yaw_sign').value)
        yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').value)
        self._yaw = wrap_to_pi(math.radians(yaw_sign * self._imu_yaw_deg + yaw_offset_deg))

    def _on_set_heading(self, heading: Float64) -> None:
        if self._imu_yaw_deg is None:
            self.get_logger().warn('viz/set_heading ignored: no imu/rpy received yet')
            return
        yaw_sign = float(self.get_parameter('yaw_sign').value)
        offset_deg = wrap_to_180(heading.data - yaw_sign * self._imu_yaw_deg)
        self.set_parameters([Parameter('yaw_offset_deg', value=offset_deg)])
        self._update_yaw()
        self.get_logger().info(
            f'Heading set to {heading.data:.1f} deg (yaw_offset_deg = {offset_deg:.1f})'
        )

    def _on_uwb_position(self, uwb: PointStamped) -> None:
        # Read every time so `ros2 param set` takes effect live
        alpha = min(max(float(self.get_parameter('position_alpha').value), 0.01), 1.0)
        if self._xy is None:
            self._xy = (uwb.point.x, uwb.point.y)
        else:
            self._xy = (
                alpha * uwb.point.x + (1.0 - alpha) * self._xy[0],
                alpha * uwb.point.y + (1.0 - alpha) * self._xy[1],
            )

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
        msg.pose.position.x = self._xy[0]
        msg.pose.position.y = self._xy[1]
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
