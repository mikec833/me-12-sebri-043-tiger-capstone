"""
go_to_goal: point to point navigation on the single EKF (brief section 5).

Subscribes  /odometry/filtered, /localization/uwb_pose, /localization/wheel_odom,
            /goal_pose (geometry_msgs/PoseStamped, frame map or odom: identical),
            /drive/armed, /drive/owner, /drive/status
Publishes   /cmd_vel (20 Hz, only while active), /nav/status (std_msgs/String),
            /set_pose (heading alignment)
Services    ~/stop (std_srvs/Trigger); ~/start_route is added by routes
Calls       /teensy_interface_node/arm and /disarm

States
    IDLE -> ARMING -> ALIGN (if not aligned) -> ROTATE <-> DRIVE -> ARRIVED
    -> next goal or IDLE. Every ROTATE <-> DRIVE switch passes through PAUSE
    (0.3 s of exactly zero command) so the firmware reversal gate sees a clean
    stop. Any state can go to ABORT: zero cmd_vel, disarm, back to IDLE.

Abort conditions: filtered pose older than max_pose_age_s, UWB older than
max_fix_age_s, position std above max_position_std, outside the geofence or
predicted outside within fence_lookahead_s, no progress (rho not reduced by
progress_distance in progress_window_s), per goal timeout, keeper takeover
(owner BLE), unexpected disarm, or any FAULT. The node never re-arms on its
own after an abort; only a new goal or route request arms again.

/nav/status lines: "<STATE>,<detail>", e.g. "DRIVE,rho=2.31", "ABORT,owner BLE".
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_navigation_pkg import nav_math as nm
from robot_navigation_pkg.align_heading import declare_align_params
from robot_navigation_pkg.drive_client import DriveClient
from robot_navigation_pkg.localization_client import LocalizationClient

RATE_HZ = 20.0
ACTIVE = ('ARMING', 'ALIGN', 'ROTATE', 'DRIVE', 'PAUSE', 'ARRIVED', 'DWELL')
NAV_PARAMS = {
    'v_max': 0.3, 'v_min': 0.05, 'omega_max': 0.6, 'omega_min': 0.2, 'omega_drive': 0.4,
    'k_rho': 0.5, 'k_psi': 1.0, 'k_omega': 1.0,
    'rotate_to_drive_deg': 20.0, 'drive_to_rotate_deg': 50.0, 'arrive_radius': 0.4,
    'pause_s': 0.3, 'accel_v': 0.3, 'accel_omega': 1.5,
    'max_pose_age_s': 0.5, 'max_fix_age_s': 2.0, 'max_position_std': 0.5,
    'fence_lookahead_s': 0.5, 'progress_distance': 0.2, 'progress_window_s': 15.0,
    'goal_timeout_s': 120.0, 'arm_timeout_s': 3.0, 'align_max_age_s': 900.0,
    'settle_after_align_s': 0.5, 'armed_grace_s': 1.0,
}


class GoToGoal(Node):

    def __init__(self, node_name='go_to_goal'):
        super().__init__(node_name)
        for name, default in NAV_PARAMS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('geofence', [0.0])   # flat x, y list; [0.0] = no fence
        self.declare_parameter('require_geofence', True)
        self.p = {k: float(self.get_parameter(k).value) for k in NAV_PARAMS}
        fence = list(self.get_parameter('geofence').value)
        self.fence = nm.pairs(fence) if len(fence) >= 6 else []
        self.require_fence = bool(self.get_parameter('require_geofence').value)
        if not self.fence:
            self.get_logger().warn(
                'No geofence configured (config/nav.yaml geofence): goals are refused'
                if self.require_fence else 'No geofence configured: driving without one')

        self.aligner = declare_align_params(self)
        self.drive = DriveClient(self, on_status=self._on_drive_status)
        self.loc = LocalizationClient(
            self, self.get_parameter('set_pose_topic').value, on_fix=self.aligner.add_fix)
        self.progress = nm.ProgressMonitor(self.p['progress_distance'], self.p['progress_window_s'])

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/nav/status', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_service(Trigger, '~/stop', self._srv_stop)

        self.state = 'IDLE'
        self.detail = ''
        self.goal = None            # (x, y, name)
        self.goal_started = None
        self.aligned_at = None      # monotonic time of the last accepted alignment
        self.state_since = time.monotonic()
        self.after_pause = None
        self.v = self.w = 0.0
        self._arm_result = None
        self._armed_at = None       # monotonic time the arm service succeeded
        self._last_status = 0.0
        self.create_timer(1.0 / RATE_HZ, self.tick)

    # ---- inputs ------------------------------------------------------------------

    def _on_goal(self, msg):
        if msg.header.frame_id not in ('map', 'odom', ''):
            self.get_logger().warn(f'goal frame {msg.header.frame_id} treated as map')
        self.start_goal(msg.pose.position.x, msg.pose.position.y, 'goal_pose')

    def _on_drive_status(self, line):
        if line.startswith('FAULT') and self.state in ACTIVE:
            self.abort(f'firmware {line}')

    def _srv_stop(self, request, response):
        self.on_stop()
        self.abort('stop requested')
        response.success, response.message = True, 'stopped'
        return response

    def on_stop(self):
        """Hook for subclasses (routes) to cancel their own state."""

    # ---- goals -----------------------------------------------------------------

    def start_goal(self, x, y, name):
        if self.require_fence and not self.fence:
            self.set_status(self.state, f'goal {name} refused: no geofence configured')
            return False
        if self.fence and not nm.inside_polygon(x, y, self.fence):
            self.set_status('IDLE' if self.state == 'IDLE' else self.state,
                            f'goal {name} ({x:.2f}, {y:.2f}) is outside the geofence, ignored')
            return False
        self.goal = (x, y, name)
        self.goal_started = time.monotonic()
        self.progress.reset(None, None)
        if self.state in ('IDLE',):
            self.enter('ARMING')
            self._arm_result = None
            self.drive.arm(lambda ok, m: setattr(self, '_arm_result', (ok, m)))
        elif self.state in ('ROTATE', 'DRIVE', 'ARRIVED', 'DWELL'):
            self.pause_then('ROTATE')
        return True

    def goal_reached(self):
        """Hook: routes override this to move to the next waypoint."""
        self.finish('arrived at ' + self.goal[2])

    # ---- state helpers ---------------------------------------------------------

    def enter(self, state, detail=''):
        self.state, self.state_since = state, time.monotonic()
        self.set_status(state, detail)

    def set_status(self, state, detail):
        self.detail = detail
        self.status_pub.publish(String(data=f'{state},{detail}'))
        if detail:
            self.get_logger().info(f'{state}: {detail}')
        self._last_status = time.monotonic()

    def pause_then(self, state):
        self.v = self.w = 0.0
        self.after_pause = state
        self.enter('PAUSE', f'then {state}')

    def publish_cmd(self, v, w, dt=1.0 / RATE_HZ, exact=False):
        if exact:
            self.v, self.w = v, w
        else:
            self.v = nm.slew(self.v, v, self.p['accel_v'], dt)
            self.w = nm.slew(self.w, w, self.p['accel_omega'], dt)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(self.v), float(self.w)
        self.cmd_pub.publish(msg)

    def abort(self, reason):
        if self.state in ACTIVE:
            self.publish_cmd(0.0, 0.0, exact=True)
            self.drive.disarm()
        self.goal = None
        self.after_pause = None
        self.enter('ABORT', reason)
        self.enter('IDLE', 'after abort')

    def finish(self, text):
        self.publish_cmd(0.0, 0.0, exact=True)
        self.drive.disarm()
        self.goal = None
        self.enter('IDLE', text)

    @property
    def aligned(self):
        return (self.aligned_at is not None
                and time.monotonic() - self.aligned_at < self.p['align_max_age_s'])

    # ---- safety -----------------------------------------------------------------

    def safety_problem(self):
        p, loc = self.p, self.loc
        if self.state != 'ARMING':
            if self.drive.owner == 'BLE':
                return 'keeper took over (owner BLE)'
            # The latched /drive/armed can arrive just after the arm reply.
            settled = (self._armed_at is None
                       or time.monotonic() - self._armed_at > p['armed_grace_s'])
            if not self.drive.armed and settled:
                return f'unexpected disarm (owner {self.drive.owner})'
        if loc.pose_age() > p['max_pose_age_s']:
            return f'filtered pose stale ({loc.pose_age():.2f} s)'
        if loc.fix_age() > p['max_fix_age_s']:
            return f'UWB stale ({loc.fix_age():.1f} s)'
        if loc.position_std() > p['max_position_std']:
            return f'position std {loc.position_std():.2f} m'
        if self.fence:
            if not nm.inside_polygon(loc.x, loc.y, self.fence):
                return f'outside geofence at ({loc.x:.2f}, {loc.y:.2f})'
            if loc.yaw is not None:
                t = p['fence_lookahead_s']
                px = loc.x + loc.v * math.cos(loc.yaw) * t
                py = loc.y + loc.v * math.sin(loc.yaw) * t
                if not nm.inside_polygon(px, py, self.fence):
                    return f'predicted outside geofence at ({px:.2f}, {py:.2f})'
        if self.goal_started and time.monotonic() - self.goal_started > p['goal_timeout_s']:
            return f'goal timeout ({p["goal_timeout_s"]:.0f} s)'
        return None

    # ---- main loop ---------------------------------------------------------------

    def tick(self):
        now = time.monotonic()
        if self.state not in ACTIVE:
            return
        problem = self.safety_problem()
        if problem:
            self.abort(problem)
            return
        handler = getattr(self, 'state_' + self.state.lower())
        handler(now)
        if now - self._last_status > 0.5:
            self.set_status(self.state, self.detail)

    def state_arming(self, now):
        self.publish_cmd(0.0, 0.0, exact=True)
        if self._arm_result is None:
            if now - self.state_since > self.p['arm_timeout_s']:
                self.abort('arm timeout')
            return
        ok, message = self._arm_result
        if ok:
            self._armed_at = now
        if not ok:
            self.abort(f'arm failed: {message}')
        elif self.aligned:
            self.pause_then('ROTATE')
        else:
            self.aligner.reset()
            self.enter('ALIGN', 'driving straight to find heading')

    def state_align(self, now):
        out = self.aligner.step(now, self.loc.travelled)
        if out[0] == 'drive':
            self.publish_cmd(out[1], 0.0)
        elif out[0] == 'stop':
            self.publish_cmd(0.0, 0.0, exact=True)
        elif out[0] == 'done':
            _, psi, length, rms = out
            self.loc.set_heading(psi, float(self.get_parameter('align_yaw_variance').value))
            self.aligned_at = now
            self.progress.reset(None, None)
            self.set_status('ALIGN', f'heading {math.degrees(psi):.1f} deg from {length:.2f} m, '
                                     f'residual {rms:.3f} m')
            self.pause_then('ROTATE')
            self.state_since = now + self.p['settle_after_align_s']  # let the EKF settle
        else:
            self.abort(f'alignment failed: {out[1]}')

    def state_pause(self, now):
        self.publish_cmd(0.0, 0.0, exact=True)
        if now - self.state_since >= self.p['pause_s']:
            nxt, self.after_pause = self.after_pause, None
            self.enter(nxt or 'ROTATE')

    def _errors(self):
        gx, gy, _ = self.goal
        return nm.goal_errors(self.loc.x, self.loc.y, self.loc.yaw, gx, gy)

    def _check_progress(self, now, rho):
        if self.progress.stalled(now, rho):
            self.abort(f'no progress: rho {rho:.2f} m not reduced by '
                       f'{self.p["progress_distance"]} m in {self.p["progress_window_s"]:.0f} s')
            return False
        return True

    def state_rotate(self, now):
        rho, e = self._errors()
        if rho < self.p['arrive_radius']:
            self.enter('ARRIVED', f'rho={rho:.2f}')
            return
        if not self._check_progress(now, rho):
            return
        if abs(e) < math.radians(self.p['rotate_to_drive_deg']):
            self.pause_then('DRIVE')
            return
        _, w = nm.rotate_command(e, self.p['k_omega'], self.p['omega_max'], self.p['omega_min'])
        self.publish_cmd(0.0, w)
        self.detail = f'e_psi={math.degrees(e):.0f}deg'

    def state_drive(self, now):
        rho, e = self._errors()
        if rho < self.p['arrive_radius']:
            self.enter('ARRIVED', f'rho={rho:.2f}')
            return
        if not self._check_progress(now, rho):
            return
        if abs(e) > math.radians(self.p['drive_to_rotate_deg']):
            self.pause_then('ROTATE')
            return
        v, w = nm.drive_command(rho, e, self.p['k_rho'], self.p['k_psi'], self.p['v_min'],
                                self.p['v_max'], self.p['omega_drive'])
        self.publish_cmd(v, w)
        self.detail = f'rho={rho:.2f} e_psi={math.degrees(e):.0f}deg'

    def state_arrived(self, now):
        self.publish_cmd(0.0, 0.0, exact=True)
        self.goal_reached()

    def state_dwell(self, now):
        self.publish_cmd(0.0, 0.0, exact=True)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.state in ACTIVE:
            node.publish_cmd(0.0, 0.0, exact=True)
            node.drive.disarm()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
