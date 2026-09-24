#!/usr/bin/env python3
"""
wasd_teleop.py

Minimal WASD keyboard teleop -> geometry_msgs/Twist on /cmd_vel. A
plain alternative to `ros2 run teleop_twist_keyboard teleop_twist_keyboard`,
whose i/j/k/l/u/o/m/,/. layout is easy to mis-type. This robot is a
differential drive (see cmd_vel_bridge.py: only linear.x and angular.z
are used, no strafing), so this node only exposes what actually does
something:

    w : drive forward
    s : drive backward
    a : rotate left in place
    d : rotate right in place
    space or k : stop
    (any other key) : stop

    + or = : increase both speeds by 10%
    - or _ : decrease both speeds by 10%

    CTRL-C : quit (publishes a final stop command first)

Each keypress publishes one Twist; the robot keeps doing whatever that
last Twist said (cmd_vel_bridge.py -> teensy_interface_node.py holds
the last REF until a new one arrives) until you press another key.
Releasing the key does NOT stop the robot -- press space/k, or Ctrl-C.

Parameters
    linear_speed_m_s      float   0.15
    angular_speed_rad_s   float   1.0
"""

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

MOVE_BINDINGS = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    ' ': (0, 0),
    'k': (0, 0),
}
SPEED_BINDINGS = {
    '+': 1.1,
    '=': 1.1,
    '-': 0.9,
    '_': 0.9,
}

INSTRUCTIONS = """
wasd_teleop -- WASD keyboard control (Ctrl-C to quit)
---------------------------------------------------
   w : forward        a : rotate left
   s : backward        d : rotate right
   space / k : stop
   +/- : speed up/down
"""


def read_key() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class WasdTeleop(Node):
    def __init__(self) -> None:
        super().__init__('wasd_teleop')

        self.declare_parameter('linear_speed_m_s', 0.40)
        self.declare_parameter('angular_speed_rad_s', 1.0)
        self.linear_speed = float(self.get_parameter('linear_speed_m_s').value)
        self.angular_speed = float(self.get_parameter('angular_speed_rad_s').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish(self, linear_dir: float, angular_dir: float) -> None:
        msg = Twist()
        msg.linear.x = linear_dir * self.linear_speed
        msg.angular.z = angular_dir * self.angular_speed
        self.pub.publish(msg)

    def stop(self) -> None:
        self.publish(0.0, 0.0)

    def run(self) -> None:
        print(INSTRUCTIONS)
        print(f'speed: linear={self.linear_speed:.2f} m/s, angular={self.angular_speed:.2f} rad/s')
        try:
            while rclpy.ok():
                key = read_key()
                if key == '\x03':  # Ctrl-C
                    break
                if key in MOVE_BINDINGS:
                    linear_dir, angular_dir = MOVE_BINDINGS[key]
                    self.publish(linear_dir, angular_dir)
                elif key in SPEED_BINDINGS:
                    factor = SPEED_BINDINGS[key]
                    self.linear_speed *= factor
                    self.angular_speed *= factor
                    print(
                        f'speed: linear={self.linear_speed:.2f} m/s, '
                        f'angular={self.angular_speed:.2f} rad/s'
                    )
                else:
                    self.stop()
        finally:
            self.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WasdTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
