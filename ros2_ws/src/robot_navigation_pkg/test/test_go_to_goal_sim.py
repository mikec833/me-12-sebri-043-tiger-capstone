"""
Closed loop simulation of go_to_goal (no ROS needed).

A kinematic robot with a first order speed lag, an "EKF" that reports the
true position but an unknown heading until set_pose arrives, noisy UWB fixes
and a fake teensy_interface_node (arm and disarm services, latched armed and
owner, cmd_vel timeout) run around the real GoToGoal node.
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import rclpy_stub  # noqa: E402

DT = 0.05


class FakeTime:
    t = 1000.0

    @classmethod
    def monotonic(cls):
        return cls.t


class World:
    def __init__(self, x=2.0, y=2.0, yaw_deg=70.0, overrides=None, seed=0):
        overrides = dict({'require_geofence': False}, **(overrides or {}))
        self.bus = rclpy_stub.install(lambda: FakeTime.t, overrides)
        from robot_navigation_pkg import go_to_goal, localization_client
        go_to_goal.time = FakeTime
        localization_client.time = FakeTime
        self.mod = go_to_goal
        self.rng = random.Random(seed)
        self.x, self.y, self.yaw = x, y, math.radians(yaw_deg)
        self.bias = -self.yaw          # EKF heading starts at 0: unknown
        self.v = self.w = 0.0
        self.armed, self.owner = False, 'NONE'
        self.arm_calls = self.disarm_calls = 0
        self.uwb_on, self.stuck = True, False
        self.pending_armed_at = None
        self.cmd = (0.0, 0.0, -1.0)
        self.zero_runs, self._zero_start = [], None
        self.states = []
        b = self.bus
        b.services['/teensy_interface_node/arm'] = self._srv_arm
        b.services['/teensy_interface_node/disarm'] = self._srv_disarm
        b.subs['/set_pose'].append(self._on_set_pose)
        b.subs['/cmd_vel'].append(self._on_cmd)
        b.subs['/nav/status'].append(lambda m: self.states.append(m.data.split(',')[0]))
        self.node = go_to_goal.GoToGoal()
        self.transitions = []                      # (state, time)
        enter = self.node.enter

        self.entered = []                          # (state, detail) per transition

        def logged_enter(state, detail=''):
            self.transitions.append((state, FakeTime.t))
            self.entered.append((state, detail))
            enter(state, detail)
        self.node.enter = logged_enter
        self.cmds = []                             # (time, v, w, node state)
        self._latch()
        self._next_fix = 0.0

    # fake teensy_interface_node
    def _latch(self):
        S = sys.modules['std_msgs.msg']
        self.bus.deliver('/drive/armed', S.Bool(data=self.armed))
        self.bus.deliver('/drive/owner', S.String(data=self.owner))

    def _srv_arm(self, req):
        self.arm_calls += 1
        if self.owner == 'BLE':
            return rclpy_stub.Msg(success=False, message='BUSY,BLE')
        self.pending_armed_at = FakeTime.t + 0.2   # latched state lags the reply
        return rclpy_stub.Msg(success=True, message='armed')

    def _srv_disarm(self, req):
        self.disarm_calls += 1
        if self.owner == 'SERIAL':
            self.armed, self.owner = False, 'NONE'
            self._latch()
        return rclpy_stub.Msg(success=True, message='K sent')

    def keeper_takeover(self):
        self.armed, self.owner = False, 'BLE'
        self._latch()

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z, FakeTime.t)
        if hasattr(self, 'cmds'):
            self.cmds.append((FakeTime.t, msg.linear.x, msg.angular.z, self.node.state))
        zero = msg.linear.x == 0.0 and msg.angular.z == 0.0
        if zero and self._zero_start is None:
            self._zero_start = FakeTime.t
        elif not zero and self._zero_start is not None:
            self.zero_runs.append(FakeTime.t - self._zero_start)
            self._zero_start = None

    def _on_set_pose(self, msg):
        q = msg.pose.pose.orientation
        self.bias = 2.0 * math.atan2(q.z, q.w) - self.yaw

    def goal(self, x, y):
        G = sys.modules['geometry_msgs.msg']
        m = G.PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = x, y
        self.bus.deliver('/goal_pose', m)

    def step(self, seconds):
        N = sys.modules['nav_msgs.msg']
        G = sys.modules['geometry_msgs.msg']
        for _ in range(int(round(seconds / DT))):
            FakeTime.t += DT
            if self.pending_armed_at and FakeTime.t >= self.pending_armed_at:
                self.armed, self.owner, self.pending_armed_at = True, 'SERIAL', None
                self._latch()
            v_c, w_c, t_c = self.cmd
            if not (self.armed and self.owner == 'SERIAL') or FakeTime.t - t_c > 0.3:
                v_c = w_c = 0.0
            if self.stuck:
                v_c = w_c = 0.0
            a = DT / 0.3
            self.v += a * (v_c - self.v)
            self.w += a * (w_c - self.w)
            self.x += self.v * math.cos(self.yaw) * DT
            self.y += self.v * math.sin(self.yaw) * DT
            self.yaw += self.w * DT

            odom = N.Odometry()
            odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
            h = self.yaw + self.bias
            odom.pose.pose.orientation.x = odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(h / 2), math.cos(h / 2)
            odom.pose.covariance = [0.01 if i in (0, 7) else 0.0 for i in range(36)]
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.v, self.w
            self.bus.deliver('/odometry/filtered', odom)
            wheel = N.Odometry()
            wheel.header.stamp.sec, wheel.header.stamp.nanosec = int(FakeTime.t), int((FakeTime.t % 1) * 1e9)
            wheel.twist.twist.linear.x = self.v
            self.bus.deliver('/localization/wheel_odom', wheel)
            if self.uwb_on and FakeTime.t >= self._next_fix:
                self._next_fix = FakeTime.t + 0.17
                fix = G.PoseWithCovarianceStamped()
                fix.pose.pose.position.x = self.x + self.rng.gauss(0, 0.08)
                fix.pose.pose.position.y = self.y + self.rng.gauss(0, 0.08)
                self.bus.deliver('/localization/uwb_pose', fix)
            for _period, cb in self.bus.timers:
                cb()

    def run_until(self, state, limit):
        t_end = FakeTime.t + limit
        while FakeTime.t < t_end:
            self.step(DT)
            if self.node.state == state:
                return True
        return False

    def status(self):
        return [m.data for m in self.bus.published['/nav/status']]


def test_reaches_5m_goal_from_unknown_heading():
    w = World(yaw_deg=70.0)
    w.goal(2.0 + 5.0, 2.0 + 0.5)   # about 5 m away, not in the initial facing
    assert w.run_until('IDLE', 120.0), w.status()[-5:]
    status = w.status()
    assert any(s.startswith('IDLE,arrived') for s in status), status[-5:]
    assert math.hypot(w.x - 7.0, w.y - 2.5) < 0.4
    for needed in ('ARMING', 'ALIGN', 'PAUSE', 'ROTATE', 'DRIVE', 'ARRIVED'):
        assert needed in w.states, needed
    # heading after alignment within a few degrees of truth
    assert abs(math.remainder(w.bias, 2 * math.pi)) < math.radians(8)
    # every ROTATE <-> DRIVE switch passes through a PAUSE of at least 0.3 s
    # with exactly zero command throughout
    seq = [st for st, _ in w.transitions if st in ('ROTATE', 'DRIVE', 'PAUSE')]
    for a, b in zip(seq, seq[1:]):
        assert not ({a, b} == {'ROTATE', 'DRIVE'}), seq
    pauses = [(t, w.transitions[i + 1][1]) for i, (st, t) in enumerate(w.transitions[:-1])
              if st == 'PAUSE']
    assert pauses and min(t1 - t0 for t0, t1 in pauses) >= 0.3 - 1e-6
    assert all(v == 0.0 and om == 0.0 for t, v, om, st in w.cmds if st == 'PAUSE')
    assert w.disarm_calls == 1 and not w.armed


def test_second_goal_skips_alignment():
    w = World(yaw_deg=-30.0)
    w.goal(5.0, 1.0)
    assert w.run_until('IDLE', 120.0)
    aligns = [st for st, _ in w.entered].count('ALIGN')
    assert aligns == 1
    w.goal(3.0, 4.0)
    assert w.run_until('IDLE', 120.0)
    assert [st for st, _ in w.entered].count('ALIGN') == aligns
    assert math.hypot(w.x - 3.0, w.y - 4.0) < 0.4


def test_keeper_takeover_aborts_and_never_rearms():
    w = World()
    w.goal(8.0, 2.0)
    assert w.run_until('DRIVE', 60.0)
    w.step(1.0)
    arms = w.arm_calls
    w.keeper_takeover()
    w.step(0.1)
    assert w.node.state == 'IDLE'
    assert any('keeper took over' in s for s in w.status())
    assert w.cmd[0] == 0.0 and w.cmd[1] == 0.0
    w.step(5.0)
    assert w.arm_calls == arms and w.node.state == 'IDLE'


def test_uwb_loss_aborts():
    w = World()
    w.goal(8.0, 2.0)
    assert w.run_until('DRIVE', 60.0)
    w.uwb_on = False
    w.step(2.5)
    assert w.node.state == 'IDLE'
    assert any('UWB stale' in s for s in w.status())


def test_no_progress_aborts():
    w = World()
    w.goal(8.0, 2.0)
    assert w.run_until('DRIVE', 60.0)
    w.stuck = True
    w.step(17.0)
    assert any('no progress' in s for s in w.status())


def test_geofence():
    fence = [0.0, 0.0, 6.0, 0.0, 6.0, 6.0, 0.0, 6.0]
    w = World(x=3.0, y=3.0, yaw_deg=0.0, overrides={'geofence': fence})
    w.goal(9.0, 3.0)                       # outside: refused
    w.step(0.5)
    assert w.node.state == 'IDLE' and w.arm_calls == 0
    assert any('outside the geofence' in s for s in w.status())
    w.x = 4.8                              # facing the east edge, 1.2 m away
    w.goal(4.0, 3.0)                       # inside, but the alignment drive heads out
    w.run_until('IDLE', 60.0)
    assert any('geofence' in s for s in w.status()[-3:]), w.status()[-3:]
    assert w.x < 6.0                       # stopped by the look ahead, before the edge


def test_no_fence_refuses_goals_by_default():
    w = World(overrides={'require_geofence': True})
    w.goal(5.0, 2.0)
    w.step(1.0)
    assert w.node.state == 'IDLE' and w.arm_calls == 0
    assert any('no geofence' in s for s in w.status())


def test_stop_service():
    w = World()
    w.goal(8.0, 2.0)
    assert w.run_until('DRIVE', 60.0)
    resp = w.bus.services['/go_to_goal/stop'](rclpy_stub.Msg())
    assert resp.success and w.node.state == 'IDLE' and w.disarm_calls >= 1


def write_waypoints(tmp_path, mode, names, dwell=(1.0, 3.0), loops=1):
    pts = {'a': [3.0, 3.0], 'b': [9.0, 3.0], 'c': [9.0, 8.0], 'd': [3.0, 8.0]}
    text = 'waypoints:\n' + ''.join(f'  {k}: [{v[0]}, {v[1]}]\n' for k, v in pts.items())
    text += (f'route:\n  mode: {mode}\n  points: [{", ".join(names)}]\n'
             f'  dwell_s: [{dwell[0]}, {dwell[1]}]\n  loops: {loops}\n')
    path = tmp_path / 'waypoints.yaml'
    path.write_text(text)
    return pts, str(path)


def visited(w):
    """Waypoint names in the order they were reached (one per transition)."""
    return [d.split('at ', 1)[1].split(' ')[0] for st, d in w.entered
            if (st == 'DWELL' and d.startswith('at ')) or (st == 'IDLE' and d.startswith('arrived at '))]


def test_shuffled_route_twice_without_abort(tmp_path):
    pts, path = write_waypoints(tmp_path, 'shuffled', [], loops=2)
    fence = [0.0, 0.0, 12.0, 0.0, 12.0, 11.0, 0.0, 11.0]
    w = World(x=6.0, y=5.0, overrides={'waypoints_file': path, 'geofence': fence,
                                        'require_geofence': True})
    w.node.rng.seed(4)
    resp = w.bus.services['/go_to_goal/start_route'](rclpy_stub.Msg())
    assert resp.success, resp.message
    assert w.run_until('IDLE', 600.0), w.status()[-4:]
    assert not any(s.startswith('ABORT') for s in w.status())
    order = visited(w)
    assert len(order) == 8 and sorted(order[:4]) == sorted(pts) == sorted(order[4:])
    assert order[4] != order[3]                 # new session does not start where it ended
    assert w.entered[-1][1].endswith('(route complete)')
    assert [st for st, _ in w.entered].count('ALIGN') == 1   # aligned once for the route


def test_sequence_route_order(tmp_path):
    _, path = write_waypoints(tmp_path, 'sequence', ['b', 'c', 'a'], dwell=(0.5, 0.5))
    w = World(x=6.0, y=5.0, overrides={'waypoints_file': path})
    assert w.bus.services['/go_to_goal/start_route'](rclpy_stub.Msg()).success
    assert w.run_until('IDLE', 400.0)
    assert visited(w) == ['b', 'c', 'a']


def test_route_refuses_points_outside_fence(tmp_path):
    _, path = write_waypoints(tmp_path, 'sequence', [])
    fence = [0.0, 0.0, 8.0, 0.0, 8.0, 11.0, 0.0, 11.0]   # b and c are outside
    w = World(overrides={'waypoints_file': path, 'geofence': fence})
    resp = w.bus.services['/go_to_goal/start_route'](rclpy_stub.Msg())
    assert not resp.success and 'outside the geofence' in resp.message
    assert w.arm_calls == 0


def test_abort_cancels_route(tmp_path):
    _, path = write_waypoints(tmp_path, 'sequence', ['b', 'c'])
    w = World(x=6.0, y=5.0, overrides={'waypoints_file': path})
    assert w.bus.services['/go_to_goal/start_route'](rclpy_stub.Msg()).success
    assert w.run_until('DRIVE', 60.0)
    w.keeper_takeover()
    w.step(0.1)
    assert w.node.state == 'IDLE' and w.node.route == [] and w.node.route_info is None
    arms = w.arm_calls
    w.step(10.0)
    assert w.arm_calls == arms
