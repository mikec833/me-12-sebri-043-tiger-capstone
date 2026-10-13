#!/usr/bin/env python3
"""
calibrate.py: drive calibration values from a calibration_drive bag.

    python3 calibrate.py <bag> [--out calib.yaml]

Computes, following the navigation brief section 2.2:
    r_eff = d_UWB / mean wheel angle change          (straight segments)
    b_eff = r_eff (dtheta_R - dtheta_L) / dpsi_IMU   (pivot segments)
    p0, kp from a least squares fit of command percent against steady
    state |wheel rate|, p = p0 + kp |w|              (level segments)

and prints a teensy_interface_node block to paste into the shared setup file
(ballrobot_pkg/config/<name>.yaml). Segments come from the BEGIN/END markers
on /calibration/segment; command percent from /drive/command.

Needs topics /wheel_states (with position), /localization/uwb_pose,
/imu/rpy, /drive/command, /calibration/segment. Reads bags with rosbag2_py
when ROS 2 is sourced, else with the pure Python 'rosbags' package.
"""

import argparse
import math
import os
import sys
from collections import Counter

import numpy as np

TOPICS = ['/wheel_states', '/localization/uwb_pose', '/imu/rpy', '/drive/command',
          '/calibration/segment']
FIX_WINDOW_S = 2.0     # stationary UWB fix: median over this window
STEADY_TAIL_S = 1.5    # steady state: last part of each level
SETTLE_LIMIT_S = 3.0   # wait at most this long for the wheels to stop


def read_bag(path):
    """{topic: [(t_receive, msg)]} for TOPICS."""
    out = {t: [] for t in TOPICS}
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        classes = {t: get_message(types[t]) for t in TOPICS if t in types}
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic in classes:
                out[topic].append((t * 1e-9, deserialize_message(data, classes[topic])))
        return out
    except ImportError:
        pass
    from pathlib import Path

    from rosbags.highlevel import AnyReader
    with AnyReader([Path(path)]) as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS]
        for conn, t, raw in reader.messages(connections=conns):
            out[conn.topic].append((t * 1e-9, reader.deserialize(raw, conn.msgtype)))
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def segments(markers):
    """[(kind, value, t_begin, t_end)] from BEGIN,<kind>[,value] / END,<kind>."""
    out, open_ = [], {}
    for t, msg in markers:
        parts = msg.data.split(',')
        if parts[0] == 'BEGIN' and len(parts) >= 2:
            open_[parts[1]] = (float(parts[2]) if len(parts) > 2 else None, t)
        elif parts[0] == 'END' and len(parts) >= 2 and parts[1] in open_:
            value, t0 = open_.pop(parts[1])
            out.append((parts[1], value, t0, t))
    return out


class Wheels:
    def __init__(self, msgs):
        msgs = [m for _, m in msgs if len(m.position) >= 2 and len(m.velocity) >= 2]
        msgs.sort(key=stamp)
        names = list(msgs[0].name) if msgs else ['left_wheel', 'right_wheel']
        il, ir = names.index('left_wheel'), names.index('right_wheel')
        self.t = np.array([stamp(m) for m in msgs])
        self.pos = np.array([[m.position[il], m.position[ir]] for m in msgs])
        self.vel = np.array([[m.velocity[il], m.velocity[ir]] for m in msgs])

    def angle(self, t):
        return np.array([np.interp(t, self.t, self.pos[:, i]) for i in (0, 1)])

    def stopped_after(self, t):
        """First time >= t when both wheels are below 0.2 rad/s."""
        idx = np.nonzero((self.t >= t) & (np.abs(self.vel).max(axis=1) < 0.2))[0]
        return min(self.t[idx[0]], t + SETTLE_LIMIT_S) if len(idx) else t + SETTLE_LIMIT_S


def uwb_fix(uwb_t, uwb_xy, t0, t1):
    sel = (uwb_t >= t0) & (uwb_t <= t1)
    return np.median(uwb_xy[sel], axis=0) if sel.sum() >= 3 else None


def command_percent(commands, t0, t1):
    lines = [m.data for t, m in commands if t0 <= t <= t1 and m.data.startswith(('M,', 'P,'))]
    if not lines:
        return None
    line = Counter(lines).most_common(1)[0][0]
    _, left, right = line.split(',')
    return 0.5 * (abs(float(left)) + abs(float(right)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--out', help='also write the YAML block to this file')
    args = ap.parse_args(argv)

    data = read_bag(args.bag)
    segs = segments(data['/calibration/segment'])
    if not segs:
        sys.exit('no BEGIN/END markers on /calibration/segment')
    wheels = Wheels(data['/wheel_states'])
    uwb = sorted(data['/localization/uwb_pose'], key=lambda x: stamp(x[1]))
    uwb_t = np.array([stamp(m) for _, m in uwb])
    uwb_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in uwb])
    rpy = sorted(data['/imu/rpy'], key=lambda x: stamp(x[1]))
    yaw_t = np.array([stamp(m) for _, m in rpy])
    yaw = np.unwrap(np.radians([m.vector.z for _, m in rpy])) if rpy else np.array([])

    report, results = [], {}

    # r_eff
    r_values = []
    for kind, _, t0, t_end in segs:
        if kind != 'straight':
            continue
        t1 = wheels.stopped_after(t_end)
        a = uwb_fix(uwb_t, uwb_xy, t0 - FIX_WINDOW_S, t0)
        b = uwb_fix(uwb_t, uwb_xy, t1, t1 + FIX_WINDOW_S)
        if a is None or b is None:
            report.append(f'straight at {t0:.1f}: no stationary UWB fix, skipped')
            continue
        d = float(np.hypot(*(b - a)))
        dtheta = float(np.mean(np.abs(wheels.angle(t1) - wheels.angle(t0))))
        if d < 3.0:
            report.append(f'straight at {t0:.1f}: only {d:.2f} m (brief asks for at least 3 m)')
        r_values.append((d, dtheta))
        report.append(f'straight: d_UWB {d:.3f} m, mean dtheta {dtheta:.2f} rad, '
                      f'r = {d / dtheta:.4f} m')
    if r_values:
        results['r_eff'] = sum(d for d, _ in r_values) / sum(th for _, th in r_values)

    # b_eff
    b_values = []
    r_eff = results.get('r_eff')
    for kind, _, t0, t_end in segs:
        if kind != 'pivot':
            continue
        if r_eff is None or not len(yaw):
            report.append('pivot: needs r_eff (a straight segment) and /imu/rpy, skipped')
            break
        t1 = wheels.stopped_after(t_end)
        dpsi = float(np.interp(t1, yaw_t, yaw) - np.interp(t0, yaw_t, yaw))
        dl, dr = wheels.angle(t1) - wheels.angle(t0)
        b = r_eff * (dr - dl) / dpsi
        b_values.append(b)
        report.append(f'pivot: dpsi {math.degrees(dpsi):.0f} deg ({abs(dpsi) / (2 * math.pi):.2f} '
                      f'turns), dtheta_R - dtheta_L {dr - dl:.2f} rad, b = {b:.4f} m')
        if abs(dpsi) < 6 * math.pi:
            report.append('  fewer than 3 turns (brief asks for at least 3)')
        if b < 0:
            report.append('  NEGATIVE: IMU yaw and wheel turn disagree in sign. Check '
                          'imu_adapter yaw_sign and the wheel/motor polarity before using this.')
    if b_values:
        results['b_eff'] = abs(float(np.mean(b_values)))

    # p0, kp
    pairs, stalled = [], []
    for kind, _, t0, t1 in segs:
        if kind != 'level':
            continue
        pct = command_percent(data['/drive/command'], t0, t1)
        sel = (wheels.t >= t1 - STEADY_TAIL_S) & (wheels.t <= t1)
        if pct is None or sel.sum() < 5:
            continue
        w = float(np.median(np.abs(wheels.vel[sel]).mean(axis=1)))
        (pairs if w > 0.3 else stalled).append((pct, w))
    for pct, w in stalled:
        report.append(f'level {pct:.1f} %: wheels did not turn ({w:.2f} rad/s), so p0 > {pct:.1f}')
    for pct, w in pairs:
        report.append(f'level {pct:.1f} %: steady |w| {w:.2f} rad/s')
    if len(pairs) >= 3:
        p = np.array([x for x, _ in pairs])
        w = np.array([y for _, y in pairs])
        kp, p0 = np.polyfit(w, p, 1)
        resid = p - (p0 + kp * w)
        results['p0'], results['kp'] = float(p0), float(kp)
        report.append(f'fit p = p0 + kp |w|: p0 {p0:.2f} %, kp {kp:.3f} %/(rad/s), '
                      f'residual RMS {math.sqrt(float(np.mean(resid ** 2))):.2f} %')
    else:
        report.append(f'levels: {len(pairs)} usable (need 3), p0/kp not fitted')

    print('\n'.join(report))
    source = os.path.basename(os.path.normpath(args.bag))
    block = ['teensy_interface_node:', '  ros__parameters:']
    for key, fmt in (('r_eff', '.4f'), ('b_eff', '.4f'), ('p0', '.2f'), ('kp', '.3f')):
        if key in results:
            block.append(f'    {key}: {results[key]:{fmt}}   # calibrated, bag {source}')
    text = '\n'.join(block) + '\n'
    print('\n# Paste into ballrobot_pkg/config/<setup>.yaml:\n' + text)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text)
    return 0 if results else 1


if __name__ == '__main__':
    sys.exit(main())
