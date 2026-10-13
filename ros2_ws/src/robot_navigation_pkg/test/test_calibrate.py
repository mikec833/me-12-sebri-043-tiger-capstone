"""calibrate.py against a synthetic calibration bag with known values.

Skipped unless the pure Python 'rosbags' package and numpy are installed.
"""
import math
import os
import sys
from pathlib import Path

import pytest

np = pytest.importorskip('numpy')
pytest.importorskip('rosbags')
from rosbags.rosbag2 import Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import calibrate  # noqa: E402


def make_bag(path):
    ts = get_typestore(Stores.ROS2_HUMBLE); T = ts.types
    Time, Header = T['builtin_interfaces/msg/Time'], T['std_msgs/msg/Header']
    R_EFF, B_EFF, P0, KP = 0.065, 0.25, 5.0, 2.2
    rng = np.random.default_rng(1)
    T0 = 1792000000.0
    def hdr(t, f=''): return Header(stamp=Time(sec=int(t), nanosec=int(round((t % 1) * 1e9)) % 1000000000), frame_id=f)
    # build a timeline of (t, v, omega, cmdline, marker)
    dt = 0.01; t = 0.0; x = y = 0.0; psi = math.radians(30); thl = thr = 0.0
    plan = [('pause', 3, 0, 0, None)]
    plan += [('straight', 25, 0.15, 0, 'straight,0.15'), ('pause', 3, 0, 0, None)]
    plan += [('pivot', 3.3 * 2 * math.pi / 0.8, 0, 0.8, 'pivot,0.8'), ('pause', 3, 0, 0, None)]
    for p in (4, 8, 11, 14, 17):
        w = max(0.0, (p - P0) / KP); plan.append(('level', 3, w * R_EFF, 0, f'level,{p}', p))
    plan.append(('pause', 3, 0, 0, None))
    with Writer(Path(path)) as wr:
        c = {k: wr.add_connection(tp, ty, typestore=ts) for k, (tp, ty) in {
            'ws': ('/wheel_states', 'sensor_msgs/msg/JointState'), 'uwb': ('/localization/uwb_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped'),
            'rpy': ('/imu/rpy', 'geometry_msgs/msg/Vector3Stamped'), 'cmd': ('/drive/command', 'std_msgs/msg/String'),
            'seg': ('/calibration/segment', 'std_msgs/msg/String')}.items()}
        def put(k, tt, m): wr.write(c[k], int((T0 + tt) * 1e9), ts.serialize_cdr(m, m.__msgtype__))
        k = 0
        for step in plan:
            kind, dur, v, om, marker = step[:5]
            if marker: put('seg', t, T['std_msgs/msg/String'](data='BEGIN,' + marker))
            n = int(round(dur / dt))
            for i in range(n):
                wl = (v - om * B_EFF / 2) / R_EFF; wrr = (v + om * B_EFF / 2) / R_EFF
                thl += wl * dt; thr += wrr * dt
                x += v * math.cos(psi) * dt; y += v * math.sin(psi) * dt; psi += om * dt
                t += dt; k += 1
                put('ws', t, T['sensor_msgs/msg/JointState'](header=hdr(T0 + t), name=['left_wheel', 'right_wheel'], position=np.array([thl, thr]), velocity=np.array([wl, wrr]) + rng.normal(0, 0.05, 2), effort=np.array([])))
                put('rpy', t, T['geometry_msgs/msg/Vector3Stamped'](header=hdr(T0 + t, 'imu_link'), vector=T['geometry_msgs/msg/Vector3'](x=179.0, y=0.0, z=(math.degrees(psi) + 180) % 360 - 180)))
                if k % 17 == 0:
                    put('uwb', t, T['geometry_msgs/msg/PoseWithCovarianceStamped'](header=hdr(T0 + t, 'map'), pose=T['geometry_msgs/msg/PoseWithCovariance'](pose=T['geometry_msgs/msg/Pose'](position=T['geometry_msgs/msg/Point'](x=x + rng.normal(0, .05), y=y + rng.normal(0, .05), z=0.2), orientation=T['geometry_msgs/msg/Quaternion'](x=0., y=0., z=0., w=1.)), covariance=np.zeros(36))))
                if k % 5 == 0 and kind != 'pause':
                    if kind == 'level': line = f'M,{step[5]:.1f},{step[5]:.1f}'
                    elif kind == 'pivot': line = 'P,-9.0,9.0'
                    else: line = 'M,10.0,10.0'
                    put('cmd', t, T['std_msgs/msg/String'](data=line))
            if marker: put('seg', t, T['std_msgs/msg/String'](data='END,' + marker.split(',')[0]))



def test_recovers_known_values(tmp_path, capsys):
    bag = tmp_path / 'calib'
    make_bag(str(bag))
    out = tmp_path / 'calib.yaml'
    assert calibrate.main([str(bag), '--out', str(out)]) == 0
    values = {}
    for line in out.read_text().splitlines()[2:]:
        key, rest = line.strip().split(':', 1)
        values[key] = float(rest.split('#')[0])
    assert abs(values['r_eff'] - 0.065) < 0.002
    assert abs(values['b_eff'] - 0.25) < 0.01
    assert abs(values['p0'] - 5.0) < 0.3
    assert abs(values['kp'] - 2.2) < 0.1
    assert 'p0 > 4.0' in capsys.readouterr().out
