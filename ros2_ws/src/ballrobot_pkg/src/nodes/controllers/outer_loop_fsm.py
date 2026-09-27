#!/usr/bin/env python3
"""
Bare-bones outer loop: waypoint FSM (STOP / R / SLT) -> Twist on cmd_vel.

    uwb/position (x, y) --+
                          +--> [FSM + controller] --> cmd_vel --> cmd_vel_bridge --> Teensy
    imu/data (yaw)     ---+

FSM (from FSM_0824 sketch):
    E  = enabled            (std_msgs/Bool on outer_loop/enable, starts False)
    A  = aligned            (|heading error| < align_tol_rad)
    WR = waypoint reached   (distance to current target < waypoint_tol_m)
    G  = final goal reached (WR on the last waypoint)

    STOP -> R    : E, A', WR', G'
    STOP -> SLT  : E, A,  WR', G'
    R    -> SLT  : E, A,  WR', G'
    SLT  -> R    : E, A', WR', G'
    SLT  -> STOP : E, G
    STOP -> STOP : E' or G

Waypoints are a flat list [x0, y0, x1, y1, ...]. The robot starts at
waypoint 0 (in STOP), so the first target is waypoint 1. Each "line" runs
from the previous waypoint to the current target.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool

from ballrobot_pkg.msg import ImuRvc


def wrap(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class OuterLoopFsm(Node):

    def __init__(self):
        super().__init__('outer_loop_fsm')

        self.declare_parameter('waypoints', [0.0, 0.0, 1.0, 0.0])
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('align_tol_rad', 0.15)   # ~8.6 deg
        self.declare_parameter('waypoint_tol_m', 0.10)
        self.declare_parameter('k_rot', 1.5)            # R:   w = k_rot * heading error
        self.declare_parameter('k_x', 0.8)              # SLT: v = k_x * distance to go
        self.declare_parameter('k_y', 3.0)              # SLT: w = -k_y * y_L - k_phi * heading error
        self.declare_parameter('k_phi', 2.0)
        self.declare_parameter('v_max', 0.3)            # m/s

        flat = list(self.get_parameter('waypoints').value)
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.target = 1  # index of the waypoint we're driving to

        self.state = 'STOP'
        self.enabled = False
        self.x = None
        self.y = None
        self.phi = None

        self.create_subscription(PointStamped, 'uwb/position', self.on_uwb, qos_profile_sensor_data)
        self.create_subscription(ImuRvc, 'imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, 'outer_loop/enable', self.on_enable, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.step)

    # ---------------- inputs ----------------

    def on_uwb(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y

    def on_imu(self, msg):
        # TODO: verify before running. This assumes IMU yaw is CCW-positive
        # and 0 deg points along the UWB +x axis. If not, the robot will
        # turn the wrong way or aim at the wrong angle.
        self.phi = math.radians(msg.yaw)

    def on_enable(self, msg):
        self.enabled = msg.data

    # ---------------- main loop ----------------

    def step(self):
        if self.x is None or self.phi is None:
            return  # no pose yet

        # Current line: previous waypoint -> target waypoint
        x0, y0 = self.waypoints[self.target - 1]
        x1, y1 = self.waypoints[self.target]
        phi_line = math.atan2(y1 - y0, x1 - x0)
        line_len = math.hypot(x1 - x0, y1 - y0)

        # Robot position in the line frame (lecture slide 22/23)
        dx, dy = self.x - x0, self.y - y0
        x_L = math.cos(phi_line) * dx + math.sin(phi_line) * dy   # progress along line
        y_L = -math.sin(phi_line) * dx + math.cos(phi_line) * dy  # sideways deviation

        heading_err = wrap(phi_line - self.phi)

        # FSM signals
        E = self.enabled
        A = abs(heading_err) < self.get_parameter('align_tol_rad').value
        WR = math.hypot(x1 - self.x, y1 - self.y) < self.get_parameter('waypoint_tol_m').value
        G = WR and self.target == len(self.waypoints) - 1

        # Intermediate waypoint reached: advance to the next one.
        # The next tick then sees a new line (probably A') and rotates.
        if WR and not G:
            self.target += 1
            return

        # Transitions
        if not E or G:
            self.state = 'STOP'
        elif not A and self.state in ('STOP', 'SLT'):
            self.state = 'R'
        elif A and self.state in ('STOP', 'R'):
            self.state = 'SLT'

        # Controllers
        v, w = 0.0, 0.0
        if self.state == 'R':
            # Pure rotation (slide 17): w = K * (phi_ref - phi)
            w = self.get_parameter('k_rot').value * heading_err
        elif self.state == 'SLT':
            # Line progress: P on distance remaining, so it comes to rest at the end
            v = self.get_parameter('k_x').value * (line_len - x_L)
            v = max(0.0, min(v, self.get_parameter('v_max').value))
            # Line deviation: PD on y_L (heading error acts as the D term,
            # since dy_L/dt = v * heading offset)
            w = (-self.get_parameter('k_y').value * y_L
                 + self.get_parameter('k_phi').value * heading_err)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = OuterLoopFsm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop on exit
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
