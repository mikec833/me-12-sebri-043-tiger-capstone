"""
calibration_drive: scripted manoeuvres for drive calibration (brief 2.2).

Run on the ground, inside UWB coverage, with a keeper watching and the
website open to stop it (Stop sends K). Record a bag at the same time:

    ros2 bag record -o calib_<date> /wheel_states /uwb/position \\
        /localization/uwb_pose /imu/rpy /drive/command /drive/status \\
        /calibration/segment
    ros2 run robot_navigation_pkg calibration_drive --ros-args -p mode:=all

then compute the values with scripts/calibrate.py.

Modes
    straight  drive straight at straight_v until UWB shows straight_distance
              (at least 3 m) or straight_timeout_s
    pivot     pivot at pivot_omega until the IMU yaw has turned pivot_turns
              (at least 3 full turns) or pivot_timeout_s
    levels    hold each speed in level_speeds for level_hold_s
    all       straight, pivot, levels

Each manoeuvre is bracketed by "BEGIN,<kind>" and "END,<kind>" on
/calibration/segment (std_msgs/String), with a stationary pause before and
after so calibrate.py can take clean UWB fixes. The drive mapping is not yet
calibrated, so speeds are only nominal; calibrate.py uses what was measured.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3Stamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from robot_navigation_pkg.drive_client import DriveClient


class Aborted(Exception):
    pass


class CalibrationDrive(Node):

    def __init__(self):
        super().__init__('calibration_drive')
        self.declare_parameter('mode', 'all')
        self.declare_parameter('straight_v', 0.15)          # m/s, nominal
        self.declare_parameter('straight_distance', 3.5)    # m, from UWB
        self.declare_parameter('straight_timeout_s', 40.0)
        self.declare_parameter('pivot_omega', 0.8)          # rad/s, nominal
        self.declare_parameter('pivot_turns', 3.2)
        self.declare_parameter('pivot_timeout_s', 90.0)
        self.declare_parameter('level_speeds', [0.05, 0.1, 0.15, 0.2, 0.25])  # m/s
        self.declare_parameter('level_hold_s', 3.0)
        self.declare_parameter('pause_s', 3.0)              # stationary before/after
        self.p = {k: self.get_parameter(k).value for k in (
            'mode', 'straight_v', 'straight_distance', 'straight_timeout_s', 'pivot_omega',
            'pivot_turns', 'pivot_timeout_s', 'level_speeds', 'level_hold_s', 'pause_s')}

        self.drive = DriveClient(self)
        self._cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._seg = self.create_publisher(String, '/calibration/segment', 10)
        self._uwb = None
        self._yaw_deg = None
        self._yaw_unwrapped = 0.0
        self.create_subscription(
            PoseWithCovarianceStamped, '/localization/uwb_pose', self._uwb_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/imu/rpy', self._rpy_cb, qos_profile_sensor_data)

    def _uwb_cb(self, msg):
        self._uwb = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _rpy_cb(self, msg):
        yaw = msg.vector.z
        if self._yaw_deg is not None:
            d = (yaw - self._yaw_deg + 180.0) % 360.0 - 180.0
            self._yaw_unwrapped += math.radians(d)
        self._yaw_deg = yaw

    # ---- helpers (run on the sequence thread) ---------------------------

    def _check(self):
        if not self.drive.driving_ok:
            raise Aborted(f'lost control (armed={self.drive.armed}, owner={self.drive.owner}, '
                          f'fault={self.drive.last_fault})')

    def _send(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self._cmd.publish(msg)

    def _hold(self, v, omega, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._check()
            if until is not None and until():
                return True
            self._send(v, omega)
            time.sleep(0.05)
        return until is None

    def _marker(self, text):
        self._seg.publish(String(data=text))
        self.get_logger().info(f'segment {text}')

    def _arm(self):
        result = {}
        done = threading.Event()

        def cb(ok, message):
            result.update(ok=ok, message=message)
            done.set()
        self.drive.arm(cb)
        if not done.wait(3.0) or not result.get('ok'):
            raise Aborted(f'arm failed: {result.get("message", "no reply")}')
        time.sleep(0.3)  # let /drive/armed and /drive/owner arrive

    # ---- manoeuvres ----------------------------------------------------------

    def straight(self):
        if self._uwb is None:
            raise Aborted('no /localization/uwb_pose; straight needs UWB')
        self._hold(0.0, 0.0, self.p['pause_s'])
        start = self._uwb
        self._marker(f'BEGIN,straight,{self.p["straight_v"]}')
        reached = self._hold(
            self.p['straight_v'], 0.0, self.p['straight_timeout_s'],
            until=lambda: math.dist(start, self._uwb) >= self.p['straight_distance'])
        self._marker('END,straight')
        self._hold(0.0, 0.0, self.p['pause_s'])
        if not reached:
            self.get_logger().warn('straight timed out before the target distance')

    def pivot(self):
        if self._yaw_deg is None:
            raise Aborted('no /imu/rpy; pivot needs the IMU')
        self._hold(0.0, 0.0, self.p['pause_s'])
        start = self._yaw_unwrapped
        target = 2.0 * math.pi * self.p['pivot_turns']
        self._marker(f'BEGIN,pivot,{self.p["pivot_omega"]}')
        self._hold(0.0, self.p['pivot_omega'], self.p['pivot_timeout_s'],
                   until=lambda: abs(self._yaw_unwrapped - start) >= target)
        self._marker('END,pivot')
        self._hold(0.0, 0.0, self.p['pause_s'])

    def levels(self):
        self._hold(0.0, 0.0, self.p['pause_s'])
        for v in self.p['level_speeds']:
            self._marker(f'BEGIN,level,{v}')
            self._hold(v, 0.0, self.p['level_hold_s'])
            self._marker('END,level')
        self._hold(0.0, 0.0, self.p['pause_s'])

    def run(self):
        mode = self.p['mode']
        steps = {'straight': [self.straight], 'pivot': [self.pivot], 'levels': [self.levels],
                 'all': [self.straight, self.pivot, self.levels]}.get(mode)
        if steps is None:
            self.get_logger().error(f'unknown mode {mode}')
            return
        try:
            self._arm()
            for step in steps:
                step()
            self.get_logger().info('calibration manoeuvres finished')
        except Aborted as exc:
            self.get_logger().error(f'aborted: {exc}')
        finally:
            self._send(0.0, 0.0)
            self.drive.disarm()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationDrive()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        time.sleep(1.0)  # discover topics and services
        node.run()
        time.sleep(0.5)  # let the disarm request go out
    except KeyboardInterrupt:
        node._send(0.0, 0.0)
        node.drive.disarm()
        time.sleep(0.3)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
