"""
Shared robot setup files (ballrobot_pkg/config/<name>.yaml).

One file per physical setup (anchor survey, calibration) is read by both
packages, so the anchor layout is written down exactly once:

    ballrobot_pkg  bringup.launch.py config:=<name>      sensor nodes
    this package   localization.launch.py config:=<name> adapters, evaluate.py

Each top level key is a node name and a node only reads its own block.
From the uwb_node block this package takes anchor_positions, and from the
teensy_interface_node block r_eff and b_eff (drive geometry). Optional
uwb_pose_adapter, imu_adapter and wheel_odom_adapter blocks override
config/adapters.yaml for that setup.

No ROS imports except the optional package lookup, so evaluate.py can use
this on a laptop.
"""

import os
from typing import List, Optional

import yaml

SENSOR_PKG = 'ballrobot_pkg'


def config_dir() -> Optional[str]:
    """Installed ballrobot_pkg/config, or the source tree copy, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(SENSOR_PKG), 'config')
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        src = os.path.join(here, '..', '..', SENSOR_PKG, 'config')
        return os.path.normpath(src) if os.path.isdir(src) else None


def resolve(config: str) -> str:
    """Same rule as bringup.launch.py: a bare name means
    ballrobot_pkg/config/<name>.yaml, anything with a '/' or a .yaml
    suffix is a path. Raises FileNotFoundError listing what exists."""
    config = config.strip()
    if '/' in config or config.endswith('.yaml'):
        path = os.path.abspath(os.path.expanduser(config))
    else:
        d = config_dir()
        path = os.path.join(d, f'{config}.yaml') if d else config
    if not os.path.isfile(path):
        d = config_dir()
        names = sorted(f[:-5] for f in os.listdir(d) if f.endswith('.yaml')) if d else []
        raise FileNotFoundError(
            f"Setup config '{config}' not found ({path}). Available: {', '.join(names)}")
    return path


def load(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def node_params(cfg: dict, node: str) -> dict:
    return dict((cfg.get(node) or {}).get('ros__parameters') or {})


def anchor_positions(cfg: dict) -> Optional[List[float]]:
    """uwb_node anchor_positions (9 floats, m) or None if the file has none."""
    flat = node_params(cfg, 'uwb_node').get('anchor_positions')
    if flat is None:
        return None
    flat = [float(v) for v in flat]
    if len(flat) != 9:
        raise ValueError(f'uwb_node anchor_positions needs 9 values, got {len(flat)}')
    return flat


def drive_geometry(cfg: dict) -> dict:
    """wheel_odom_adapter overrides from the teensy_interface_node block."""
    drive = node_params(cfg, 'teensy_interface_node')
    out = {}
    if 'r_eff' in drive:
        out['wheel_radius'] = float(drive['r_eff'])
    if 'b_eff' in drive:
        out['b_eff'] = float(drive['b_eff'])
    return out
