"""
align_heading: one heading alignment drive, then exit (brief 4.1).

Arms the drive, drives straight for d_align, fits a line through the UWB
fixes and sends the heading to the EKF on set_pose, then disarms. Useful
for localization only runs; go_to_goal runs the same alignment itself.
Start the robot facing open ground: the drive direction is unknown.

    ros2 run robot_navigation_pkg align_heading --ros-args \\
        --params-file $(ros2 pkg prefix robot_navigation_pkg)/share/robot_navigation_pkg/config/nav.yaml
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from robot_navigation_pkg.drive_client import DriveClient
from robot_navigation_pkg.localization_client import LocalizationClient
from robot_navigation_pkg.nav_math import HeadingAligner

ALIGN_PARAMS = {'v_align': 0.15, 'd_align': 1.5, 'align_min_length': 1.0,
                'align_max_rms': 0.15, 'align_yaw_variance': 0.05}


def declare_align_params(node):
    for name, default in ALIGN_PARAMS.items():
        node.declare_parameter(name, default)
    node.declare_parameter('set_pose_topic', '/set_pose')
    g = node.get_parameter
    return HeadingAligner(v_align=g('v_align').value, d_align=g('d_align').value,
                          min_length=g('align_min_length').value,
                          max_rms=g('align_max_rms').value)


class AlignHeading(Node):

    def __init__(self):
        super().__init__('align_heading')
        self.aligner = declare_align_params(self)
        self.drive = DriveClient(self)
        self.loc = LocalizationClient(
            self, self.get_parameter('set_pose_topic').value, on_fix=self.aligner.add_fix)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state = 'WAIT'
        self.finished = False
        self.create_timer(0.05, self.tick)

    def publish(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd.publish(msg)

    def finish(self, ok, text):
        self.publish(0.0, 0.0)
        self.drive.disarm()
        (self.get_logger().info if ok else self.get_logger().error)(text)
        self.state, self.finished = 'DONE', True

    def tick(self):
        if self.state == 'WAIT':
            if self.loc.has_pose and self.loc.fix_age() < 2.0:
                self.state = 'ARMING'
                self.drive.arm(self._armed)
            return
        if self.state != 'ALIGN':
            return
        if not self.drive.driving_ok:
            self.finish(False, f'lost control (owner {self.drive.owner})')
            return
        out = self.aligner.step(self.get_clock().now().nanoseconds * 1e-9, self.loc.travelled)
        if out[0] == 'drive':
            self.publish(out[1], 0.0)
        elif out[0] == 'stop':
            self.publish(0.0, 0.0)
        elif out[0] == 'done':
            _, psi, length, rms = out
            self.loc.set_heading(psi, self.get_parameter('align_yaw_variance').value)
            self.finish(True, f'heading {math.degrees(psi):.1f} deg from {length:.2f} m '
                              f'(residual {rms:.3f} m), sent to the EKF')
        else:
            self.finish(False, f'alignment failed: {out[1]}')

    def _armed(self, ok, message):
        if ok:
            self.aligner.reset()
            self.state = 'ALIGN'
        else:
            self.finish(False, f'arm failed: {message}')


def main(args=None):
    rclpy.init(args=args)
    node = AlignHeading()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(5):  # flush the disarm request
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.publish(0.0, 0.0)
        node.drive.disarm()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
