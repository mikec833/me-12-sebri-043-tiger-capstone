import math

from ballrobot_localization_pkg import geometry as g


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_euler_identity_and_yaw():
    assert close(g.euler_deg_to_quaternion(0, 0, 0), (0, 0, 0, 1))
    s = math.sqrt(0.5)
    assert close(g.euler_deg_to_quaternion(0, 0, 90), (0, 0, s, s))
    assert close(g.euler_deg_to_quaternion(180, 0, 0), (1, 0, 0, 0))


def test_body_angular_velocity_level():
    q0 = g.euler_deg_to_quaternion(0, 0, 10)
    q1 = g.euler_deg_to_quaternion(0, 0, 11)
    w = g.body_angular_velocity(q0, q1, 0.1)
    assert close(w, (0, 0, math.radians(10)), 1e-9)


def test_body_angular_velocity_upside_down():
    # Sensor rolled 180 deg: a positive world yaw rate is negative about sensor z.
    q0 = g.euler_deg_to_quaternion(180, 0, 10)
    q1 = g.euler_deg_to_quaternion(180, 0, 11)
    w = g.body_angular_velocity(q0, q1, 0.1)
    assert close(w, (0, 0, -math.radians(10)), 1e-9)


def test_body_angular_velocity_wraps():
    q0 = g.euler_deg_to_quaternion(0, 0, 179.5)
    q1 = g.euler_deg_to_quaternion(0, 0, -179.5)
    w = g.body_angular_velocity(q0, q1, 1.0)
    assert close(w, (0, 0, math.radians(1.0)), 1e-9)
    assert g.body_angular_velocity(q0, q1, 0.0) is None


def test_plane_distance_matches_clamp_signature():
    plane = g.plane_from_points((0, 0, 1.3), (1.8, 0, 1.3), (0, 1.8, 1.3))
    assert g.distance_to_plane((-1.797, 0.768, 1.3), plane) < 1e-12
    assert g.distance_to_plane((0.5, 0.9, 0.11), plane) > 1.0
    assert g.plane_from_points((0, 0, 0), (1, 0, 0), (2, 0, 0)) is None


def test_windows():
    w = g.pairs_from_flat([30.0, 40.0, 0.0, 0.0])
    assert g.in_any_window(30.0, w) and g.in_any_window(39.9, w)
    assert not g.in_any_window(40.0, w)
    assert not g.in_any_window(0.0, w)  # zero length window is a no-op
    try:
        g.pairs_from_flat([1.0])
        assert False
    except ValueError:
        pass


def test_diff_drive():
    v, wz = g.diff_drive_twist(10.0, 10.0, 1.0, 0.072, 0.22)
    assert abs(v - 0.72) < 1e-12 and wz == 0.0
    v, wz = g.diff_drive_twist(-5.0, 5.0, 1.0, 0.072, 0.22)
    assert v == 0.0 and abs(wz - 0.072 * 10 / 0.22) < 1e-12


def test_covariance_helpers():
    c = g.diagonal_covariance([1, 2, 3])
    assert c == [1, 0, 0, 0, 2, 0, 0, 0, 3]
    assert g.covariance_is_unset([0.0] * 9)
    assert g.covariance_is_unset([-1.0] + [0.0] * 8)
    assert not g.covariance_is_unset(c)


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
