"""
uwb_pose_adapter

Turns the trilaterated UWB tag position from ballrobot_pkg/uwb_node.py
into a pose that robot_localization can fuse.

Subscribes
    /uwb/position              geometry_msgs/PointStamped (input_topic)
Publishes
    /localization/uwb_pose     geometry_msgs/PoseWithCovarianceStamped
    /diagnostics               counters (received, published, drop reasons)

What it adds
    * Covariance: diagonal uwb_var_x, uwb_var_y, uwb_var_z, and
      orientation_variance on roll, pitch and yaw (the pose carries no
      orientation, so these are made huge and the EKF only fuses x, y, z).
    * frame_id: set to frame_id (map) whatever the source says.
    * Clamped rejection: uwb_node.py does not publish the clamped flag.
      A clamped solve forces z^2 = 0 in uwb_trilateration.py, which puts
      the point exactly on the anchor plane, so a sample is treated as
      clamped when its distance to the plane through anchor_positions is
      below clamp_tolerance. Keep anchor_positions equal to the uwb_node
      parameter of the same name.
    * NaN or infinite coordinates are dropped.
    * Dropout test: samples whose stamp falls in any dropout window are
      suppressed (see dropout_windows below).

The source header.stamp is copied unchanged. Samples with a zero stamp
are dropped and reported, never restamped.
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ballrobot_localization_pkg.counters import CounterPublisher, stamp_is_zero, stamp_to_sec
from ballrobot_localization_pkg.geometry import (
    diagonal_covariance, distance_to_plane, in_any_window, pairs_from_flat,
    plane_from_points)


class UwbPoseAdapter(Node):

    def __init__(self):
        super().__init__('uwb_pose_adapter')
        self.declare_parameter('input_topic', '/uwb/position')
        self.declare_parameter('output_topic', '/localization/uwb_pose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('uwb_var_x', 0.05)
        self.declare_parameter('uwb_var_y', 0.05)
        self.declare_parameter('uwb_var_z', 1.0)
        self.declare_parameter('orientation_variance', 1e6)
        self.declare_parameter('infer_clamped', True)
        self.declare_parameter(
            'anchor_positions', [0.0, 0.0, 1.3, 1.8, 0.0, 1.3, 0.0, 1.8, 1.3])
        self.declare_parameter('clamp_tolerance', 1e-3)
        # Flat [start, end, start, end, ...] in seconds. ROS 2 parameters
        # cannot hold nested lists. A zero length pair suppresses nothing.
        self.declare_parameter('dropout_windows', [0.0, 0.0])
        # 'relative': seconds after the stamp of the first sample received.
        # 'absolute': raw header.stamp seconds (bag time).
        self.declare_parameter('dropout_time_reference', 'relative')

        p = self.get_parameter
        self._frame_id = p('frame_id').value
        self._infer_clamped = bool(p('infer_clamped').value)
        self._clamp_tol = float(p('clamp_tolerance').value)
        self._windows = pairs_from_flat(list(p('dropout_windows').value))
        self._relative = p('dropout_time_reference').value == 'relative'
        self._t0 = None

        flat = [float(v) for v in p('anchor_positions').value]
        if len(flat) != 9:
            raise ValueError(f'anchor_positions needs 9 values, got {len(flat)}')
        self._plane = plane_from_points(
            tuple(flat[0:3]), tuple(flat[3:6]), tuple(flat[6:9]))
        if self._plane is None:
            raise ValueError('anchor_positions are collinear, cannot form a plane')

        cov = diagonal_covariance([
            p('uwb_var_x').value, p('uwb_var_y').value, p('uwb_var_z').value,
            p('orientation_variance').value, p('orientation_variance').value,
            p('orientation_variance').value])
        self._covariance = cov

        self._counters = CounterPublisher(self, [
            'received', 'published', 'dropped_clamped', 'dropped_nan',
            'dropped_zero_stamp', 'dropped_dropout'])
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, p('output_topic').value, 10)
        self.create_subscription(
            PointStamped, p('input_topic').value, self._on_point, qos_profile_sensor_data)

        windows = [w for w in self._windows if w[1] > w[0]]
        self.get_logger().info(
            f'UWB adapter up. clamped inference: {self._infer_clamped}, '
            f'dropout windows ({"relative" if self._relative else "absolute"} s): {windows}')

    def _on_point(self, msg: PointStamped) -> None:
        c = self._counters
        c.inc('received')
        if stamp_is_zero(msg.header.stamp):
            c.inc('dropped_zero_stamp')
            self.get_logger().error(
                'UWB sample with a zero header.stamp, dropped (not restamped)',
                throttle_duration_sec=5.0)
            return

        t = stamp_to_sec(msg.header.stamp)
        if self._t0 is None:
            self._t0 = t

        pt = (msg.point.x, msg.point.y, msg.point.z)
        if not all(math.isfinite(v) for v in pt):
            c.inc('dropped_nan')
            return

        if self._infer_clamped and distance_to_plane(pt, self._plane) < self._clamp_tol:
            c.inc('dropped_clamped')
            return

        if in_any_window(t - self._t0 if self._relative else t, self._windows):
            c.inc('dropped_dropout')
            return

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id
        out.pose.pose.position = msg.point
        out.pose.pose.orientation.w = 1.0
        out.pose.covariance = self._covariance
        self._pub.publish(out)
        c.inc('published')


def main(args=None):
    rclpy.init(args=args)
    node = UwbPoseAdapter()
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
