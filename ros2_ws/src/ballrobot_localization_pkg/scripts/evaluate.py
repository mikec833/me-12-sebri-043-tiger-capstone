#!/usr/bin/env python3
"""
evaluate.py: offline evaluation of a ballrobot_localization_pkg replay.

Two modes (run with -h for every option):

  run        Evaluate one bag recorded during an offline replay. Writes
             PNG figures, metrics.json and summary.md into the output folder.
  summarize  Collect the metrics.json of several runs (A, B, C, D) into one
             comparison table, results/summary.md.

The bag must contain the raw sensor topics (republished by ros2 bag play)
and the localization outputs. scripts/run_comparison.sh records the right
set. Times in this script are header stamps, never bag receive times.

Reads bags with rosbag2_py when ROS 2 is sourced, otherwise with the pure
Python 'rosbags' package (pip install rosbags), so it also runs on a
laptop without ROS.
"""

import argparse
import csv
import json
import math
import os
import sys
from bisect import bisect_left

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

try:
    import yaml
except ImportError:  # only needed to read anchors from adapters.yaml
    yaml = None

TOPICS = {
    'uwb_raw': '/uwb/position',
    'uwb_pose': '/localization/uwb_pose',
    'global': '/odometry/filtered',   # single EKF output (map = odom)
    'diag': '/diagnostics',
    'rpy': '/imu/rpy',
    'wheel': '/localization/wheel_odom',
    'imu': '/localization/imu',
}
POSE_GATE = 5.0  # must match pose0_rejection_threshold in config/ekf.yaml (x, y)
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ADAPTERS = os.path.join(HERE, '..', 'config', 'adapters.yaml')

# Colours: one fixed hue per data source, reused in every figure.
COLORS = {'uwb': '#9aa0a6', 'local': '#1f77b4', 'global': '#d62728',
          'anchor': '#2ca02c', 'dropout': '#ff7f0e'}


# bag reading

def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(path):
    """Return {key: [msg, ...]} for every key in TOPICS found in the bag."""
    wanted = {v: k for k, v in TOPICS.items()}
    out = {k: [] for k in TOPICS}
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        rosbag2_py = None

    if rosbag2_py is not None:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in wanted if t in types]))
        classes = {t: get_message(types[t]) for t in wanted if t in types}
        while reader.has_next():
            topic, data, _ = reader.read_next()
            out[wanted[topic]].append(deserialize_message(data, classes[topic]))
        return out

    from pathlib import Path

    from rosbags.highlevel import AnyReader
    with AnyReader([Path(path)]) as reader:
        conns = [c for c in reader.connections if c.topic in wanted]
        for conn, _, raw in reader.messages(connections=conns):
            out[wanted[conn.topic]].append(reader.deserialize(raw, conn.msgtype))
    return out


# series helpers

def odom_series(msgs):
    """Arrays t, xyz (n x 3), position covariance trace, yaw (rad)."""
    if not msgs:
        return None
    msgs = sorted(msgs, key=stamp)
    t = np.array([stamp(m) for m in msgs])
    xyz = np.array([[m.pose.pose.position.x, m.pose.pose.position.y,
                     m.pose.pose.position.z] for m in msgs])
    cov = np.array([np.asarray(m.pose.covariance, dtype=float).reshape(6, 6) for m in msgs])
    trace = cov[:, 0, 0] + cov[:, 1, 1] + cov[:, 2, 2]
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                   m.pose.pose.orientation.z, m.pose.pose.orientation.w] for m in msgs])
    yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]),
                     1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
    return {'t': t, 'xyz': xyz, 'trace': trace, 'cov': cov, 'yaw': yaw}


def point_series(msgs, pose=False):
    if not msgs:
        return None
    msgs = sorted(msgs, key=stamp)
    t = np.array([stamp(m) for m in msgs])
    if pose:
        xyz = np.array([[m.pose.pose.position.x, m.pose.pose.position.y,
                         m.pose.pose.position.z] for m in msgs])
    else:
        xyz = np.array([[m.point.x, m.point.y, m.point.z] for m in msgs])
    return {'t': t, 'xyz': xyz}


def interp_xyz(series, t_query):
    return np.column_stack([np.interp(t_query, series['t'], series['xyz'][:, i])
                            for i in range(3)])


def rate_hz(series):
    if series is None or len(series['t']) < 2:
        return 0.0
    span = series['t'][-1] - series['t'][0]
    return (len(series['t']) - 1) / span if span > 0 else 0.0


def load_anchors(setup, adapters_path):
    """Anchor positions (3 x 3, m) from the shared setup file, falling back
    to adapters.yaml. Same precedence as localization.launch.py."""
    if yaml is None:
        return None
    try:
        sys.path.insert(0, os.path.join(HERE, '..'))
        from ballrobot_localization_pkg import setup_config
        flat = setup_config.anchor_positions(setup_config.load(setup_config.resolve(setup)))
    except (ImportError, FileNotFoundError) as exc:
        print(f'note: setup config not used ({exc})')
        flat = None
    if flat is None and os.path.exists(adapters_path):
        with open(adapters_path) as f:
            flat = yaml.safe_load(f)['uwb_pose_adapter']['ros__parameters']['anchor_positions']
    return np.array(flat, dtype=float).reshape(3, 3) if flat else None


def parse_windows(text):
    if not text:
        return []
    v = [float(x) for x in text.split(',')]
    return [(v[i], v[i + 1]) for i in range(0, len(v) - 1, 2) if v[i + 1] > v[i]]


# metrics

def adapter_counters(diag_msgs):
    """Last counter values published by each ballrobot_localization_pkg adapter."""
    latest = {}
    for m in diag_msgs:
        for s in m.status:
            if s.name.startswith('ballrobot_localization_pkg:'):
                latest[s.hardware_id] = {kv.key: int(kv.value) for kv in s.values}
    return latest


def ekf_reject_messages(diag_msgs):
    """robot_localization /diagnostics entries from the EKF that mention rejection."""
    hits = []
    for m in diag_msgs:
        for s in m.status:
            if 'ekf' not in s.name:
                continue
            text = (s.message + ' ' + ' '.join(f'{kv.key}={kv.value}' for kv in s.values))
            if 'reject' in text.lower():
                hits.append((stamp(m), text.strip()))
    return hits


def estimated_rejections(uwb_pose_msgs, glob):
    """Mahalanobis distance of each fused UWB sample against the last
    /odometry/filtered estimate stamped before it, over x and y (the fused
    UWB axes).

    This is an outside estimate of what pose0_rejection_threshold rejects:
    robot_localization gates against its predicted state at the exact sample
    time, which this script cannot see.
    """
    if not uwb_pose_msgs or glob is None:
        return None
    rejected, d_all = [], []
    for m in sorted(uwb_pose_msgs, key=stamp):
        t = stamp(m)
        i = bisect_left(glob['t'], t) - 1
        if i < 0:
            continue
        r = np.array([m.pose.pose.position.x, m.pose.pose.position.y,
                      m.pose.pose.position.z]) - glob['xyz'][i]
        r = r[:2]
        s = glob['cov'][i][:2, :2] + np.asarray(m.pose.covariance, dtype=float).reshape(6, 6)[:2, :2]
        try:
            d = float(math.sqrt(r @ np.linalg.solve(s, r)))
        except np.linalg.LinAlgError:
            continue
        d_all.append(d)
        if d >= POSE_GATE:
            rejected.append((t, d))
    return {'checked': len(d_all), 'over_gate': rejected,
            'median_distance': float(np.median(d_all)) if d_all else None}


def dropout_drift(windows, t_ref, uwb, glob):
    """/odometry/filtered error against UWB at the start and end of each window.

    End error: the last global estimate stamped before the window ends
    against the first fused UWB sample after it (clamped samples are never
    in uwb_pose). Start error is the same measure going into the window, as
    a baseline.
    """
    rows = []
    if uwb is None or glob is None:
        return rows

    for start, end in windows:
        t_s, t_e = t_ref + start, t_ref + end
        row = {'window': [start, end], 'global_xy_error_start_m': None,
               'global_xy_drift_m': None, 'uwb_gap_after_end_s': None}
        before = np.nonzero(uwb['t'] < t_s)[0]
        if len(before):
            k = before[-1]
            g = interp_xyz(glob, np.array([uwb['t'][k]]))[0]
            row['global_xy_error_start_m'] = float(np.hypot(*(g[:2] - uwb['xyz'][k, :2])))
        i = bisect_left(glob['t'], t_e) - 1
        after = np.nonzero(uwb['t'] >= t_e)[0]
        if i >= 0 and len(after):
            k = after[0]
            row['global_xy_drift_m'] = float(np.hypot(*(glob['xyz'][i, :2] - uwb['xyz'][k, :2])))
            row['uwb_gap_after_end_s'] = float(uwb['t'][k] - t_e)
        rows.append(row)
    return rows


def ground_truth_rms(path, t_ref, uwb, glob):
    pts = []
    with open(path) as f:
        for row in csv.DictReader(f):
            pts.append((float(row['t']), float(row['x']), float(row['y'])))
    if not pts:
        return None
    gt = np.array(pts)
    if gt[0, 0] < 1e8:  # relative seconds, same reference as dropout windows
        gt[:, 0] += t_ref
    res = {'points': len(gt)}
    for name, s in (('uwb_raw', uwb), ('global', glob)):
        if s is None:
            continue
        inside = (gt[:, 0] >= s['t'][0]) & (gt[:, 0] <= s['t'][-1])
        if not inside.any():
            continue
        p = interp_xyz(s, gt[inside, 0])
        err = np.hypot(p[:, 0] - gt[inside, 1], p[:, 1] - gt[inside, 2])
        res[f'{name}_rms_m'] = float(np.sqrt(np.mean(err ** 2)))
    return res


def yaw_offset_estimate(uwb_pose, rpy_msgs, wheel_msgs, yaw_sign):
    """Offset between raw IMU yaw and UWB track heading (diagnostic: the single
    EKF does not fuse IMU yaw; a steady offset means alignment is consistent).

    Uses pairs of fused UWB samples about 2 s apart that are at least 0.5 m
    apart while the wheels report forward motion, and compares the track
    heading with the raw IMU yaw at the midpoint. Assumes base_link x points
    along the direction of travel and the IMU mount yaw is zero.
    """
    if uwb_pose is None or not rpy_msgs:
        return None
    rpy = sorted(rpy_msgs, key=stamp)
    rt = np.array([stamp(m) for m in rpy])
    ry = np.unwrap(np.radians([yaw_sign * m.vector.z for m in rpy]))
    if wheel_msgs:
        wheel = sorted(wheel_msgs, key=stamp)
        wt = np.array([stamp(m) for m in wheel])
        wv = np.array([m.twist.twist.linear.x for m in wheel])
    t, xyz = uwb_pose['t'], uwb_pose['xyz']
    diffs = []
    j = 0
    for i in range(len(t)):
        while j < len(t) and t[j] - t[i] < 2.0:
            j += 1
        if j >= len(t):
            break
        d = xyz[j, :2] - xyz[i, :2]
        if np.hypot(*d) < 0.5:
            continue
        mid = 0.5 * (t[i] + t[j])
        if wheel_msgs and np.interp(mid, wt, wv) <= 0.05:
            continue
        diffs.append(math.atan2(d[1], d[0]) - np.interp(mid, rt, ry))
    if len(diffs) < 5:
        return {'segments': len(diffs), 'note': 'not enough forward driving to estimate'}
    c, s = np.mean(np.cos(diffs)), np.mean(np.sin(diffs))
    spread = math.degrees(math.sqrt(max(0.0, -2 * math.log(max(math.hypot(c, s), 1e-12)))))
    return {'segments': len(diffs), 'suggested_yaw_offset_deg': math.degrees(math.atan2(s, c)),
            'circular_std_deg': spread}


def yaw_rate_check(imu_msgs, wheel_msgs, imu_roll):
    """Compare the IMU derived yaw rate with the wheel yaw rate.

    Both are fused as vyaw, so they must agree in sign. The IMU rate is in
    imu_link; with the mount roll near pi its z axis is base_link -z.
    A negative slope means yaw_sign (imu_adapter) or left_sign/right_sign
    (wheel_odom_adapter) is wrong. 1 / slope scales b_eff.
    """
    imu = [m for m in imu_msgs if m.angular_velocity_covariance[0] != -1.0]
    if not imu or not wheel_msgs:
        return None
    imu = sorted(imu, key=stamp)
    wheel = sorted(wheel_msgs, key=stamp)
    flip = -1.0 if abs(abs(imu_roll) - math.pi) < math.pi / 2 else 1.0
    it = np.array([stamp(m) for m in imu])
    iw = flip * np.array([m.angular_velocity.z for m in imu])
    wt = np.array([stamp(m) for m in wheel])
    ww = np.array([m.twist.twist.angular.z for m in wheel])
    turning = np.abs(ww) > 0.1
    if turning.sum() < 50:
        return {'samples': int(turning.sum()), 'note': 'not enough turning to compare'}
    x, y = ww[turning], np.interp(wt[turning], it, iw)
    slope = float(np.dot(x, y) / np.dot(x, x))
    return {'samples': int(turning.sum()), 'correlation': float(np.corrcoef(x, y)[0, 1]),
            'imu_over_wheel_slope': slope}


# figures

def plot_xy(out_dir, uwb, local, glob, anchors, label):
    fig, ax = plt.subplots(figsize=(7, 6))
    if uwb is not None:
        ax.scatter(uwb['xyz'][:, 0], uwb['xyz'][:, 1], s=4, color=COLORS['uwb'],
                   label='raw UWB (/uwb/position)')
    if local is not None:
        ax.plot(local['xyz'][:, 0], local['xyz'][:, 1], color=COLORS['local'], lw=1.2,
                label='/odometry/local (odom frame)')
    if glob is not None:
        ax.plot(glob['xyz'][:, 0], glob['xyz'][:, 1], color=COLORS['global'], lw=1.6,
                label='/odometry/filtered')
    if anchors is not None:
        ax.scatter(anchors[:, 0], anchors[:, 1], marker='^', s=80, color=COLORS['anchor'],
                   label='UWB anchors', zorder=5)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    ax.set_title(f'XY trajectory, run {label}')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'xy.png'), dpi=130)
    plt.close(fig)


def plot_time(out_dir, t_ref, uwb, local, glob, windows, label):
    fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for i, axis in enumerate('xyz'):
        ax = axes[i]
        if uwb is not None:
            ax.scatter(uwb['t'] - t_ref, uwb['xyz'][:, i], s=3, color=COLORS['uwb'], label='raw UWB')
        if local is not None:
            ax.plot(local['t'] - t_ref, local['xyz'][:, i], color=COLORS['local'], lw=1, label='local')
        if glob is not None:
            ax.plot(glob['t'] - t_ref, glob['xyz'][:, i], color=COLORS['global'], lw=1.3, label='filtered')
        ax.set_ylabel(f'{axis} (m)')
        ax.grid(alpha=0.3)
    if glob is not None:
        axes[3].plot(glob['t'] - t_ref, glob['trace'], color=COLORS['global'])
        axes[3].set_yscale('log')
    axes[3].set_ylabel('filtered cov trace\nxx+yy+zz (m^2)')
    axes[3].grid(alpha=0.3)
    axes[3].set_xlabel('time since first UWB sample (s)')
    for ax in axes:
        for start, end in windows:
            ax.axvspan(start, end, color=COLORS['dropout'], alpha=0.15)
    axes[0].legend(loc='upper right', fontsize=8, ncol=3)
    axes[0].set_title(f'Position and filtered covariance, run {label}')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'position_time.png'), dpi=130)
    plt.close(fig)


# modes

def run(args):
    os.makedirs(args.out, exist_ok=True)
    data = read_bag(args.bag)
    uwb = point_series(data['uwb_raw'])
    uwb_pose = point_series(data['uwb_pose'], pose=True)
    local = None  # single EKF: no separate odom-frame output
    glob = odom_series(data['global'])
    windows = parse_windows(args.dropout)
    # Same reference as uwb_pose_adapter 'relative' dropout windows.
    t_ref = uwb['t'][0] if uwb is not None else (glob or local or {'t': [0.0]})['t'][0]
    anchors = load_anchors(args.config, args.adapters_config)

    plot_xy(args.out, uwb, local, glob, anchors, args.label)
    plot_time(args.out, t_ref, uwb, local, glob, windows, args.label)

    counters = adapter_counters(data['diag'])
    uwb_c = counters.get('uwb_pose_adapter', {})
    received = uwb_c.get('received', len(data['uwb_raw']))
    metrics = {
        'label': args.label,
        'bag': os.path.abspath(args.bag),
        'rates_hz': {'uwb_raw': rate_hz(uwb), 'uwb_pose': rate_hz(uwb_pose),
                     'filtered': rate_hz(glob)},
        'adapter_counters': counters,
        'uwb_clamped_dropped': uwb_c.get('dropped_clamped'),
        'uwb_clamped_pct': (100.0 * uwb_c['dropped_clamped'] / received
                            if received and 'dropped_clamped' in uwb_c else None),
        'ekf_reject_diagnostics': ekf_reject_messages(data['diag']),
        'ekf_estimated_rejections': estimated_rejections(data['uwb_pose'], glob),
        'dropout_drift': dropout_drift(windows, t_ref, uwb_pose or uwb, glob),
        'yaw_offset': yaw_offset_estimate(uwb_pose, data['rpy'], data['wheel'], args.yaw_sign),
        'yaw_rate_check': yaw_rate_check(data['imu'], data['wheel'], args.imu_roll),
    }
    if glob is not None:
        metrics['global_final_cov_trace_m2'] = float(glob['trace'][-1])
    if args.ground_truth:
        metrics['ground_truth'] = ground_truth_rms(args.ground_truth, t_ref, uwb, glob)

    with open(os.path.join(args.out, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2, default=float)
    with open(os.path.join(args.out, 'summary.md'), 'w') as f:
        f.write(run_markdown(metrics))
    print(run_markdown(metrics))


def fmt(v, digits=3):
    if v is None:
        return 'n/a'
    if isinstance(v, float):
        return f'{v:.{digits}f}'
    return str(v)


def run_markdown(m):
    r = m['rates_hz']
    lines = [f"# Run {m['label']}", '', f"Bag: `{m['bag']}`", '',
             '![XY](xy.png)', '', '![Position and covariance](position_time.png)', '',
             '| output | rate (Hz) |', '| --- | --- |']
    lines += [f'| {k} | {fmt(v, 1)} |' for k, v in r.items()]
    lines += ['', f"UWB samples dropped as clamped: {fmt(m['uwb_clamped_dropped'])} "
                  f"({fmt(m['uwb_clamped_pct'], 1)}%)", '']
    est = m['ekf_estimated_rejections']
    if est:
        lines.append(f"UWB samples over the {POSE_GATE} gate (estimated, see README): "
                     f"{len(est['over_gate'])} of {est['checked']}, median distance "
                     f"{fmt(est['median_distance'], 2)}")
    lines.append(f"EKF /diagnostics entries mentioning rejection: "
                 f"{len(m['ekf_reject_diagnostics'])}")
    for d in m['dropout_drift']:
        lines.append(f"Dropout {d['window']} s: filtered XY error {fmt(d['global_xy_drift_m'])} m "
                     f"at the end (was {fmt(d['global_xy_error_start_m'])} m going in)")
    if m.get('ground_truth'):
        g = m['ground_truth']
        lines.append(f"Ground truth RMS ({g['points']} points): raw UWB "
                     f"{fmt(g.get('uwb_raw_rms_m'))} m, global {fmt(g.get('global_rms_m'))} m")
    y = m.get('yaw_offset')
    if y and 'suggested_yaw_offset_deg' in y:
        lines.append(f"IMU yaw minus UWB track heading: {y['suggested_yaw_offset_deg']:.1f} deg "
                     f"(circular std {y['circular_std_deg']:.1f} deg, {y['segments']} segments)")
    yc = m.get('yaw_rate_check')
    if yc and 'imu_over_wheel_slope' in yc:
        lines.append(f"Yaw rate check, IMU vs wheels ({yc['samples']} turning samples): "
                     f"correlation {yc['correlation']:+.2f}, slope {yc['imu_over_wheel_slope']:+.2f}"
                     + ('  SIGN MISMATCH: check yaw_sign or wheel signs'
                        if yc['imu_over_wheel_slope'] < 0 else ''))
    return '\n'.join(lines) + '\n'


RUNS = [
    ('A', 'on', 'on', 'Baseline'),
    ('B', 'off', 'on', 'Value of the IMU yaw rate'),
    ('C', 'on', 'off', 'Value of the encoders'),
    ('D', 'on', 'on, 10 s UWB dropout', 'Drift under canopy'),
]


def summarize(args):
    rows = ['| Run | IMU yaw rate | Wheel odom | Purpose | filtered Hz | '
            'clamped dropped | over gate (est.) | dropout drift (m) | '
            'GT RMS filtered (m) | final cov trace (m^2) |',
            '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    figures = []
    for label, yaw, odom, purpose in RUNS:
        path = os.path.join(args.results, f'run_{label}', 'metrics.json')
        if not os.path.exists(path):
            rows.append(f'| {label} | {yaw} | {odom} | {purpose} | not run | | | | | |')
            continue
        with open(path) as f:
            m = json.load(f)
        est = m.get('ekf_estimated_rejections') or {}
        drift = [d.get('global_xy_drift_m') for d in m.get('dropout_drift', [])]
        gt = (m.get('ground_truth') or {}).get('global_rms_m')
        rows.append(
            f"| {label} | {yaw} | {odom} | {purpose} | {fmt(m['rates_hz']['filtered'], 1)} | "
            f"{fmt(m.get('uwb_clamped_dropped'))} "
            f"({fmt(m.get('uwb_clamped_pct'), 1)}%) | "
            f"{len(est.get('over_gate', [])) if est else 'n/a'} | "
            f"{', '.join(fmt(d, 2) for d in drift) or 'n/a'} | {fmt(gt)} | "
            f"{fmt(m.get('global_final_cov_trace_m2'), 4)} |")
        figures += [f'### Run {label}: {purpose}', '',
                    f'![run {label} XY](run_{label}/xy.png)', '',
                    f'![run {label} position](run_{label}/position_time.png)', '']
    text = ['# ballrobot_localization_pkg comparison runs', '',
            'Generated by scripts/evaluate.py summarize. Per run details are in',
            'results/run_<label>/summary.md.', ''] + rows + [''] + figures
    with open(os.path.join(args.results, 'summary.md'), 'w') as f:
        f.write('\n'.join(text) + '\n')
    print('\n'.join(rows))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='mode', required=True)
    r = sub.add_parser('run', help='evaluate one replay bag')
    r.add_argument('--bag', required=True, help='bag recorded during offline replay')
    r.add_argument('--out', required=True, help='output folder, e.g. results/run_A')
    r.add_argument('--label', default='A')
    r.add_argument('--dropout', default='',
                   help='start,end[,start,end] seconds after the first UWB sample')
    r.add_argument('--ground-truth', default='', help='CSV with columns t,x,y')
    r.add_argument('--config', default='default',
                   help='shared setup file used for the run: name in ballrobot_pkg/config/ '
                        'or a path (bringup saves a copy in src/outputs/run_<id>/)')
    r.add_argument('--adapters-config', default=DEFAULT_ADAPTERS)
    r.add_argument('--yaw-sign', type=float, default=1.0,
                   help='imu_adapter yaw_sign used in this run')
    r.add_argument('--imu-roll', type=float, default=math.pi,
                   help='base_link to imu_link roll (rad) used in this run')
    s = sub.add_parser('summarize', help='build results/summary.md from run_A..run_D')
    s.add_argument('--results', default=os.path.join(HERE, '..', 'results'))
    args = ap.parse_args(argv)
    if args.mode == 'run':
        run(args)
    else:
        summarize(args)


if __name__ == '__main__':
    sys.exit(main())
