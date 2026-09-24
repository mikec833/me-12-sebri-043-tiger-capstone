#!/usr/bin/env python3
"""
Convert geometry_msgs/Twist on /cmd_vel to per-wheel speed setpoints (rad/s).

CSV export
    On shutdown, writes src/outputs/cmd_vel_outputs/cmd_vel_log_run<id>.csv
    (one row per Twist received: time_s, stamp_s, linear_x, angular_z,
    the resulting left/right rad/s setpoints before and after clamping
    to max_wheel_speed_rad_s). stamp_s is wall-clock receipt time as
    float epoch seconds (Twist carries no header/stamp of its own) --
    unlike time_s (seconds since this node started), it's directly
    comparable across nodes/processes, so it's what to join on when
    correlating with imu_log/uwb_log/etc. <id> is the run_id parameter
    -- shared with imu_node.py/uwb_node.py/etc. when started from
    bringup.launch.py, so their CSVs line up. See run_logging.py.

Parameters
    run_id    str    ""
"""

import csv
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from ballrobot_pkg.msg import LeftRightFloat32
from run_logging import resolve_run_id

DEFAULT_WHEEL_RADIUS_M = 0.072
DEFAULT_WHEEL_BASE_M = 0.22
DEFAULT_MAX_WHEEL_SPEED_RAD_S = 5.0  # identified max is ~13 rad/s; stay under it for PI headroom

# Relative to wherever `ros2 run`/`ros2 launch` is launched from (i.e. ros2_ws/)
OUTPUT_DIR = 'src/outputs/cmd_vel_outputs'


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('wheel_radius_m', DEFAULT_WHEEL_RADIUS_M)
        self.declare_parameter('wheel_base_m', DEFAULT_WHEEL_BASE_M)
        self.declare_parameter('max_wheel_speed_rad_s', DEFAULT_MAX_WHEEL_SPEED_RAD_S)
        # Shared across nodes in the same bringup; see run_logging.py.
        self.declare_parameter('run_id', '')

        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.max_speed = float(self.get_parameter('max_wheel_speed_rad_s').value)

        self.pub = self.create_publisher(LeftRightFloat32, 'wheel_speed_cmd', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # For the CSV export: one row per received Twist
        self.start_time = self.get_clock().now()
        self.log_time = []
        self.log_stamp_s = []
        self.log_linear_x = []
        self.log_angular_z = []
        self.log_left_unclamped = []
        self.log_right_unclamped = []
        self.log_left = []
        self.log_right = []

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z

        v_left = v - omega * self.wheel_base / 2.0
        v_right = v + omega * self.wheel_base / 2.0

        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        out = LeftRightFloat32()
        out.left = max(-self.max_speed, min(self.max_speed, omega_left))
        out.right = max(-self.max_speed, min(self.max_speed, omega_right))
        self.pub.publish(out)

        self.log_time.append(self.elapsed_seconds())
        self.log_stamp_s.append(self.get_clock().now().nanoseconds * 1e-9)
        self.log_linear_x.append(v)
        self.log_angular_z.append(omega)
        self.log_left_unclamped.append(omega_left)
        self.log_right_unclamped.append(omega_right)
        self.log_left.append(out.left)
        self.log_right.append(out.right)

    def elapsed_seconds(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def save_csv(self):
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        run_id = resolve_run_id(OUTPUT_DIR, 'cmd_vel_log_run', self.get_parameter('run_id').value)
        filename = f'cmd_vel_log_run{run_id}.csv'

        with open(os.path.join(OUTPUT_DIR, filename), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_s', 'stamp_s', 'linear_x', 'angular_z',
                'left_rad_s_unclamped', 'right_rad_s_unclamped',
                'left_rad_s', 'right_rad_s',
            ])
            writer.writerows(zip(
                self.log_time, self.log_stamp_s, self.log_linear_x, self.log_angular_z,
                self.log_left_unclamped, self.log_right_unclamped,
                self.log_left, self.log_right))

        self.get_logger().info(f'Saved {filename}')

    def destroy_node(self):
        self.save_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
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
