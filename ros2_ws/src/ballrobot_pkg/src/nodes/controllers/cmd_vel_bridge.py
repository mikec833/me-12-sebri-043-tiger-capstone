#!/usr/bin/env python3
"""Convert geometry_msgs/Twist on /cmd_vel to per-wheel speed setpoints (rad/s)."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from ballrobot_pkg.msg import LeftRightFloat32

DEFAULT_WHEEL_RADIUS_M = 0.072
DEFAULT_WHEEL_BASE_M = 0.22
DEFAULT_MAX_WHEEL_SPEED_RAD_S = 5.0  # identified max is ~13 rad/s; stay under it for PI headroom


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('wheel_radius_m', DEFAULT_WHEEL_RADIUS_M)
        self.declare_parameter('wheel_base_m', DEFAULT_WHEEL_BASE_M)
        self.declare_parameter('max_wheel_speed_rad_s', DEFAULT_MAX_WHEEL_SPEED_RAD_S)

        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.max_speed = float(self.get_parameter('max_wheel_speed_rad_s').value)

        self.pub = self.create_publisher(LeftRightFloat32, 'wheel_speed_cmd', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

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
