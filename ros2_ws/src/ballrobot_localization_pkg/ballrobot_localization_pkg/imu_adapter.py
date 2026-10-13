"""
imu_adapter

Combines the two BNO085 UART-RVC topics from ballrobot_pkg/imu_node.py
into one sensor_msgs/Imu for robot_localization.

Subscribes
    /imu/rpy      geometry_msgs/Vector3Stamped   roll, pitch, yaw in degrees
                  (fused on the BNO085, magnetometer included)
    /imu/accel    geometry_msgs/Vector3Stamped   accelerometer, raw counts
                  (milli g, gravity included)
Publishes
    /localization/imu    sensor_msgs/Imu, frame_id imu_link
    /diagnostics         counters

The two inputs come from the same serial frame and share a header, so
they are paired with message_filters ApproximateTime (slop sync_slop).

Orientation: quaternion from the RVC Euler angles (ZYX, as in
imu_quaternion_node.py), after yaw = yaw_sign * yaw + yaw_offset_deg.
yaw_offset_deg aligns magnetic heading with the map x axis.

Angular velocity: the RVC frame has no gyro output. When
derive_angular_velocity is true it is computed from the rotation between
the current orientation and the one angular_velocity_window samples
earlier, expressed in the sensor frame. This lags the true rate by about
half the window. With it off, angular_velocity_covariance[0] is -1.

Linear acceleration: counts times accel_scale (m/s^2 per count). Gravity
is still present, so the EKF must run with
imu0_remove_gravitational_acceleration: true.

Covariance: the source carries none, so every block is filled from
parameters. yaw_variance replaces only the yaw element of the orientation
covariance, to allow for magnetometer EMI from the motor driver.

The source header.stamp is copied unchanged. Zero stamps are dropped and
reported.
"""

from collections import deque

import rclpy
from geometry_msgs.msg import Vector3Stamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from ballrobot_localization_pkg.counters import CounterPublisher, stamp_is_zero, stamp_to_sec
from ballrobot_localization_pkg.geometry import (
    body_angular_velocity, diagonal_covariance, euler_deg_to_quaternion)


class ImuAdapter(Node):

    def __init__(self):
        super().__init__('imu_adapter')
        self.declare_parameter('rpy_topic', '/imu/rpy')
        self.declare_parameter('accel_topic', '/imu/accel')
        self.declare_parameter('output_topic', '/localization/imu')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('sync_slop', 0.005)
        self.declare_parameter('sync_queue_size', 20)
        self.declare_parameter('accel_scale', 9.80665e-3)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('roll_variance', 0.0012)
        self.declare_parameter('pitch_variance', 0.0012)
        self.declare_parameter('yaw_variance', 0.03)
        self.declare_parameter('derive_angular_velocity', True)
        self.declare_parameter('angular_velocity_window', 5)
        self.declare_parameter('angular_velocity_variance', 0.0025)
        self.declare_parameter('linear_acceleration_variance', 0.25)

        p = self.get_parameter
        self._frame_id = p('frame_id').value
        self._accel_scale = float(p('accel_scale').value)
        self._yaw_sign = float(p('yaw_sign').value)
        self._yaw_offset = float(p('yaw_offset_deg').value)
        self._derive_w = bool(p('derive_angular_velocity').value)
        self._history = deque(maxlen=max(2, int(p('angular_velocity_window').value) + 1))

        self._orientation_cov = diagonal_covariance([
            p('roll_variance').value, p('pitch_variance').value, p('yaw_variance').value])
        w_var = float(p('angular_velocity_variance').value)
        self._angular_cov = diagonal_covariance([w_var] * 3)
        self._accel_cov = diagonal_covariance(
            [float(p('linear_acceleration_variance').value)] * 3)
        self._no_estimate = [-1.0] + [0.0] * 8

        self._counters = CounterPublisher(
            self, ['pairs', 'published', 'dropped_zero_stamp', 'dropped_window_fill'])
        self._pub = self.create_publisher(Imu, p('output_topic').value, 10)

        rpy_sub = Subscriber(self, Vector3Stamped, p('rpy_topic').value,
                             qos_profile=qos_profile_sensor_data)
        acc_sub = Subscriber(self, Vector3Stamped, p('accel_topic').value,
                             qos_profile=qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer(
            [rpy_sub, acc_sub], int(p('sync_queue_size').value), float(p('sync_slop').value))
        self._sync.registerCallback(self._on_pair)
        self.get_logger().info('IMU adapter up')

    def _on_pair(self, rpy: Vector3Stamped, accel: Vector3Stamped) -> None:
        c = self._counters
        c.inc('pairs')
        if stamp_is_zero(rpy.header.stamp):
            c.inc('dropped_zero_stamp')
            self.get_logger().error(
                'IMU sample with a zero header.stamp, dropped (not restamped)',
                throttle_duration_sec=5.0)
            return

        yaw = self._yaw_sign * rpy.vector.z + self._yaw_offset
        q = euler_deg_to_quaternion(rpy.vector.x, rpy.vector.y, yaw)
        t = stamp_to_sec(rpy.header.stamp)

        out = Imu()
        out.header.stamp = rpy.header.stamp
        out.header.frame_id = self._frame_id
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q
        out.orientation_covariance = self._orientation_cov

        out.linear_acceleration.x = accel.vector.x * self._accel_scale
        out.linear_acceleration.y = accel.vector.y * self._accel_scale
        out.linear_acceleration.z = accel.vector.z * self._accel_scale
        out.linear_acceleration_covariance = self._accel_cov

        if self._derive_w:
            if self._history and t <= self._history[-1][0]:
                # Out of order or repeated stamp: restart the window.
                self._history.clear()
            self._history.append((t, q))
            if len(self._history) < self._history.maxlen:
                c.inc('dropped_window_fill')
                return
            t_old, q_old = self._history[0]
            w = body_angular_velocity(q_old, q, t - t_old)
            out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = w
            out.angular_velocity_covariance = self._angular_cov
        else:
            out.angular_velocity_covariance = self._no_estimate

        self._pub.publish(out)
        c.inc('published')


def main(args=None):
    rclpy.init(args=args)
    node = ImuAdapter()
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
