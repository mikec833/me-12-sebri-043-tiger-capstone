"""
Pure Python math shared by the adapter nodes.

Nothing here imports ROS, so it can be unit tested on any machine
(see test/test_geometry.py).
"""

import math
from typing import List, Optional, Sequence, Tuple

Quat = Tuple[float, float, float, float]  # (x, y, z, w)
Vec3 = Tuple[float, float, float]


# orientation

def euler_deg_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quat:
    """ZYX intrinsic Euler angles in degrees to a quaternion (x, y, z, w).

    Same convention as ballrobot_pkg/imu_quaternion_node.py: yaw about Z,
    then pitch about Y, then roll about X.
    """
    hr = math.radians(roll_deg) * 0.5
    hp = math.radians(pitch_deg) * 0.5
    hy = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(hr), math.sin(hr)
    cp, sp = math.cos(hp), math.sin(hp)
    cy, sy = math.cos(hy), math.sin(hy)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def body_angular_velocity(q_prev: Quat, q_curr: Quat, dt: float) -> Optional[Vec3]:
    """Mean body frame angular velocity (rad/s) that rotates q_prev into q_curr.

    Both quaternions give the sensor orientation in the world frame, so the
    relative rotation q_prev^-1 * q_curr is expressed in the sensor (body)
    frame, which is the frame sensor_msgs/Imu angular_velocity uses.
    Returns None when dt is not positive.
    """
    if dt <= 0.0:
        return None
    dq = quat_multiply(quat_conjugate(q_prev), q_curr)
    if dq[3] < 0.0:  # take the short way round
        dq = (-dq[0], -dq[1], -dq[2], -dq[3])
    vec_norm = math.sqrt(dq[0] ** 2 + dq[1] ** 2 + dq[2] ** 2)
    if vec_norm < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vec_norm, dq[3])
    scale = angle / (vec_norm * dt)
    return (dq[0] * scale, dq[1] * scale, dq[2] * scale)


def rotate_vector(q: Quat, v: Vec3) -> Vec3:
    """Rotate v by unit quaternion q (q v q*)."""
    p = quat_multiply(quat_multiply(q, (v[0], v[1], v[2], 0.0)), quat_conjugate(q))
    return (p[0], p[1], p[2])


def world_yaw_rate_in_sensor(yaw_rate: float, mount: Quat) -> Vec3:
    """Sensor frame vector that the static mount maps to (0, 0, yaw_rate) in
    base_link. mount is the base_link to imu_link rotation (the static TF), so
    v_base = mount * v_imu and v_imu = mount^-1 * v_base."""
    return rotate_vector(quat_conjugate(mount), (0.0, 0.0, yaw_rate))


# covariance

def diagonal_covariance(variances: Sequence[float]) -> List[float]:
    """Row major n x n matrix with the given variances on the diagonal."""
    n = len(variances)
    cov = [0.0] * (n * n)
    for i, v in enumerate(variances):
        cov[i * n + i] = float(v)
    return cov


def covariance_is_unset(cov: Sequence[float]) -> bool:
    """True when a ROS covariance array is all zero or flagged -1 (unknown)."""
    return len(cov) == 0 or cov[0] == -1.0 or all(c == 0.0 for c in cov)


# UWB

def plane_from_points(p1: Vec3, p2: Vec3, p3: Vec3) -> Optional[Tuple[Vec3, float]]:
    """Unit normal n and offset d of the plane n.x = d through three points.

    Returns None if the points are collinear.
    """
    u = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
    v = (p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2])
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    norm = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
    if norm < 1e-12:
        return None
    n = (n[0] / norm, n[1] / norm, n[2] / norm)
    return n, n[0] * p1[0] + n[1] * p1[1] + n[2] * p1[2]


def distance_to_plane(point: Vec3, plane: Tuple[Vec3, float]) -> float:
    n, d = plane
    return abs(n[0] * point[0] + n[1] * point[1] + n[2] * point[2] - d)


def in_any_window(t: float, windows: Sequence[Tuple[float, float]]) -> bool:
    """True if start <= t < end for any (start, end) pair."""
    return any(start <= t < end for start, end in windows)


def pairs_from_flat(flat: Sequence[float]) -> List[Tuple[float, float]]:
    """[a, b, c, d] to [(a, b), (c, d)]. Raises ValueError on an odd length."""
    if len(flat) % 2:
        raise ValueError(f'expected an even number of values, got {len(flat)}')
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)]


# wheel odometry

def diff_drive_twist(w_left: float, w_right: float, k_v: float,
                     wheel_radius: float, b_eff: float) -> Tuple[float, float]:
    """Body forward speed (m/s) and yaw rate (rad/s) from wheel rates (rad/s)."""
    v_x = k_v * wheel_radius * (w_left + w_right) / 2.0
    w_z = k_v * wheel_radius * (w_right - w_left) / b_eff
    return v_x, w_z
