"""
drive_command.py

Converts a body velocity command (v m/s, omega rad/s) into one firmware v4.7
drive line ("M,l,r" or "P,l,r", percent, left first). Pure Python, no ROS,
so it can be unit tested anywhere. Used by teensy_interface_node.py.

    v_L = v - omega * b_eff / 2        v_R = v + omega * b_eff / 2
    w_wheel = v_wheel / r_eff
    p = sign(w) * (p0 + kp * |w|)      for w != 0

Rules, in order (see the navigation brief, section 2.1):
  1. |v| < eps_v and omega != 0: pivot. "P,-p,p" or "P,p,-p" with exactly
     equal magnitudes (the firmware rejects P when |l + r| > 1).
  2. v != 0 and the wheel signs differ: the inner wheel is set to 0. M never
     carries opposite signs, because the firmware reversal gate would stop.
  3. Otherwise "M,pL,pR".
  4. Deadband: 0 < |p| < p0 becomes 0 below p0 / 2, else p0 with its sign.
  5. Clip to max_percent (the same value sent to the firmware with C).
A wheel ground speed below eps_v counts as zero, so rounding noise never
starts a wheel at the deadband percent.
"""

import math
from dataclasses import dataclass


@dataclass
class DriveParams:
    r_eff: float        # m, effective wheel radius against the shell
    b_eff: float        # m, effective track
    p0: float           # percent, deadband offset
    kp: float           # percent per rad/s of wheel speed
    max_percent: float  # percent, same as the C limit
    eps_v: float        # m/s, speeds below this count as zero


def _percent(wheel_speed: float, p: DriveParams) -> float:
    if abs(wheel_speed) < p.eps_v:
        return 0.0
    w = wheel_speed / p.r_eff
    percent = math.copysign(p.p0 + p.kp * abs(w), w)
    if 0.0 < abs(percent) < p.p0:
        percent = 0.0 if abs(percent) < p.p0 / 2.0 else math.copysign(p.p0, percent)
    return max(-p.max_percent, min(p.max_percent, percent))


def _fmt(x: float) -> str:
    return f'{x:.1f}' if x != 0.0 else '0'


def command_line(v: float, omega: float, p: DriveParams) -> str:
    """Return the drive line (no newline) for body velocity v, omega."""
    half_track = omega * p.b_eff / 2.0
    if abs(v) < p.eps_v:
        if abs(half_track) < p.eps_v:
            return 'M,0,0'
        mag = abs(_percent(half_track, p))
        if mag == 0.0:
            return 'M,0,0'
        # Positive omega (counter clockwise) turns the left wheel backwards.
        left, right = (-mag, mag) if omega > 0.0 else (mag, -mag)
        return f'P,{_fmt(left)},{_fmt(right)}'

    left = _percent(v - half_track, p)
    right = _percent(v + half_track, p)
    if left * right < 0.0:
        # Keep the wheel that agrees with v, stop the inner one.
        if math.copysign(1.0, left) != math.copysign(1.0, v):
            left = 0.0
        else:
            right = 0.0
    return f'M,{_fmt(left)},{_fmt(right)}'
