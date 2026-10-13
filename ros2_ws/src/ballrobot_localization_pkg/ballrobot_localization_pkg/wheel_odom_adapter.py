"""
wheel_odom_adapter

Converts the measured wheel rates from ballrobot_pkg's
teensy_interface_node.py into a body twist for robot_localization.

Subscribes
    /wheel_states            sensor_msgs/JointState, velocity in rad/s at
                             the gearbox output shaft (input_units rad_s),
                             or encoder counts/s (input_units counts_per_s)
Publishes
    /localization/wheel_odom nav_msgs/Odometry, frame_id odom,
                             child_frame_id base_link, twist only
    /diagnostics             counters

Kinematics (Sphero style differential drive inside the shell):
    v_x = k_v * wheel_radius * (w_L + w_R) / 2
    v_y = 0      non holonomic: the ball rolls along its heading
    v_z = 0      ground robot
    w_z = k_v * wheel_radius * (w_R - w_L) / b_eff

v_y = 0 is the non holonomic constraint that keeps heading observable
between UWB fixes; var_vy sets how hard the EKF holds it.

Decimation: input arrives at up to 100 Hz. Wheel rates are averaged over
each 1 / output_rate_hz interval and published once, stamped with the last
input sample in the interval (0 publishes every sample).

wheel_radius and b_eff are normally copied by localization.launch.py from
the teensy_interface_node r_eff and b_eff in the shared setup file, so the
drive and the odometry always use the same geometry. The pose block is left at zero with a huge
covariance and must not be fused. No TF is published here.

The source header.stamp is copied unchanged. Zero stamps are dropped and
reported.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ballrobot_localization_pkg.counters import CounterPublisher, stamp_is_zero
from ballrobot_localization_pkg.geometry import diagonal_covariance, diff_drive_twist


class WheelOdomAdapter(Node):

    def __init__(self):
        super().__init__('wheel_odom_adapter')
        self.declare_parameter('input_topic', '/wheel_states')
        self.declare_parameter('output_topic', '/localization/wheel_odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('left_joint', 'left_wheel')
        self.declare_parameter('right_joint', 'right_wheel')
        self.declare_parameter('input_units', 'rad_s')
        self.declare_parameter('counts_per_rev', 4480.0)
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)
        self.declare_parameter('k_v', 1.0)
        self.declare_parameter('wheel_radius', 0.072)
        self.declare_parameter('b_eff', 0.22)
        self.declare_parameter('var_vx', 0.01)
        self.declare_parameter('var_vy', 0.1)
        self.declare_parameter('var_vz', 0.01)
        self.declare_parameter('var_wz', 0.04)
        self.declare_parameter('output_rate_hz', 50.0)
        self.declare_parameter('unused_variance', 1e6)

        p = self.get_parameter
        self._frame_id = p('frame_id').value
        self._child = p('child_frame_id').value
        self._left_name = p('left_joint').value
        self._right_name = p('right_joint').value
        units = p('input_units').value
        if units == 'rad_s':
            self._to_rad = 1.0
        elif units == 'counts_per_s':
            self._to_rad = 2.0 * math.pi / float(p('counts_per_rev').value)
        else:
            raise ValueError(f"input_units must be 'rad_s' or 'counts_per_s', got {units}")
        self._left_sign = float(p('left_sign').value)
        self._right_sign = float(p('right_sign').value)
        self._k_v = float(p('k_v').value)
        self._radius = float(p('wheel_radius').value)
        self._b_eff = float(p('b_eff').value)

        big = float(p('unused_variance').value)
        self._pose_cov = diagonal_covariance([big] * 6)
        self._twist_cov = diagonal_covariance([
            p('var_vx').value, p('var_vy').value, p('var_vz').value,
            big, big, p('var_wz').value])

        rate = float(p('output_rate_hz').value)
        self._period_ns = int(1e9 / rate) if rate > 0.0 else 0
        self._acc = [0.0, 0.0, 0, None]   # sum w_L, sum w_R, count, interval start (ns)

        self._counters = CounterPublisher(
            self, ['received', 'published', 'dropped_zero_stamp', 'dropped_malformed'])
        self._pub = self.create_publisher(Odometry, p('output_topic').value, 10)
        self.create_subscription(JointState, p('input_topic').value, self._on_joints, 10)
        self.get_logger().info(
            f'Wheel odom adapter up: r={self._radius} m, b_eff={self._b_eff} m, k_v={self._k_v}')

    def _wheel_rates(self, msg: JointState):
        try:
            i_l = msg.name.index(self._left_name)
            i_r = msg.name.index(self._right_name)
            w_l = msg.velocity[i_l]
            w_r = msg.velocity[i_r]
        except (ValueError, IndexError):
            return None
        if not (math.isfinite(w_l) and math.isfinite(w_r)):
            return None
        return (self._left_sign * w_l * self._to_rad, self._right_sign * w_r * self._to_rad)

    def _on_joints(self, msg: JointState) -> None:
        c = self._counters
        c.inc('received')
        if stamp_is_zero(msg.header.stamp):
            c.inc('dropped_zero_stamp')
            self.get_logger().error(
                'wheel_states with a zero header.stamp, dropped (not restamped)',
                throttle_duration_sec=5.0)
            return
        rates = self._wheel_rates(msg)
        if rates is None:
            c.inc('dropped_malformed')
            self.get_logger().warn(
                f'wheel_states without finite velocity for {self._left_name} and '
                f'{self._right_name}, dropped', throttle_duration_sec=5.0)
            return

        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        acc = self._acc
        if acc[3] is None or t_ns < acc[3]:
            acc[:] = [0.0, 0.0, 0, t_ns]
        acc[0] += rates[0]
        acc[1] += rates[1]
        acc[2] += 1
        if t_ns - acc[3] < self._period_ns:
            return
        w_l, w_r = acc[0] / acc[2], acc[1] / acc[2]
        acc[:] = [0.0, 0.0, 0, t_ns]

        v_x, w_z = diff_drive_twist(w_l, w_r, self._k_v, self._radius, self._b_eff)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id
        out.child_frame_id = self._child
        out.pose.pose.orientation.w = 1.0
        out.pose.covariance = self._pose_cov
        out.twist.twist.linear.x = v_x
        out.twist.twist.angular.z = w_z
        out.twist.covariance = self._twist_cov
        self._pub.publish(out)
        c.inc('published')


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomAdapter()
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
