import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from robot_navigation_pkg import nav_math as nm  # noqa: E402


def test_wrap():
    assert abs(nm.wrap(3 * math.pi) - math.pi) < 1e-12
    assert abs(nm.wrap(-math.pi / 2) + math.pi / 2) < 1e-12
    assert abs(nm.wrap(2 * math.pi + 0.1) - 0.1) < 1e-12


def test_fit_heading_all_directions_and_sign():
    rng = random.Random(0)
    for deg in range(-170, 181, 35):
        psi = math.radians(deg)
        pts = [(2 + s * math.cos(psi) + rng.gauss(0, 0.05), 3 + s * math.sin(psi) + rng.gauss(0, 0.05))
               for s in [i * 0.05 for i in range(31)]]
        heading, length, rms = nm.fit_heading(pts)
        assert abs(nm.wrap(heading - psi)) < math.radians(5), deg
        assert 1.3 < length < 1.7 and rms < 0.1


def test_fit_heading_curved_path_has_large_residual():
    pts = [(math.cos(a) * 1.0, math.sin(a) * 1.0) for a in [i * 0.1 for i in range(32)]]
    assert nm.fit_heading(pts)[2] > 0.15
    assert nm.fit_heading([(0, 0), (1, 1)]) is None


def test_aligner_accepts_good_run():
    al = nm.HeadingAligner(v_align=0.15, d_align=1.5)
    t, travelled, psi = 0.0, 0.0, math.radians(40)
    while True:
        out = al.step(t, travelled)
        if out[0] == 'drive':
            travelled += out[1] * 0.05
        if out[0] in ('done', 'fail'):
            break
        al.add_fix(travelled * math.cos(psi), travelled * math.sin(psi))
        t += 0.05
    assert out[0] == 'done' and abs(out[1] - psi) < 0.01


def test_aligner_retries_then_fails_without_motion():
    al = nm.HeadingAligner(v_align=0.15, d_align=1.5)
    t, results = 0.0, []
    for _ in range(4000):
        out = al.step(t, 0.0)       # wheels report nothing: times out
        al.add_fix(0.0, 0.0)
        results.append(out[0])
        if out[0] in ('done', 'fail'):
            break
        t += 0.05
    assert out[0] == 'fail'
    assert results.count('stop') > 1  # settled, retried, settled again


def test_control_law():
    v, w = nm.rotate_command(0.05, 1.0, 0.6, 0.2)
    assert v == 0.0 and w == 0.2            # raised to omega_min
    v, w = nm.rotate_command(-3.0, 1.0, 0.6, 0.2)
    assert w == -0.6
    v, w = nm.drive_command(5.0, 0.0, 0.5, 1.0, 0.05, 0.3, 0.4)
    assert v == 0.3 and w == 0.0
    v, w = nm.drive_command(5.0, math.radians(100), 0.5, 1.0, 0.05, 0.3, 0.4)
    assert v == 0.0                         # never drive away from the goal
    rho, e = nm.goal_errors(0, 0, math.pi / 2, 1, 0)
    assert abs(rho - 1) < 1e-12 and abs(e + math.pi / 2) < 1e-12


def test_slew():
    assert nm.slew(0.0, 1.0, 0.5, 0.1) == 0.05
    assert nm.slew(0.0, -1.0, 0.5, 0.1) == -0.05
    assert nm.slew(0.98, 1.0, 0.5, 0.1) == 1.0


def test_geofence():
    square = nm.pairs([0, 0, 10, 0, 10, 10, 0, 10])
    assert nm.inside_polygon(5, 5, square)
    assert not nm.inside_polygon(11, 5, square)
    assert nm.inside_polygon(100, 100, [])


def test_progress_monitor():
    pm = nm.ProgressMonitor(0.2, 15.0)
    assert not pm.stalled(0.0, 5.0)
    assert not pm.stalled(10.0, 4.9)
    assert pm.stalled(15.5, 4.85)
    pm.reset(20.0, 4.85)
    assert not pm.stalled(30.0, 4.6)
    assert not pm.stalled(44.0, 4.5)        # progress at 30 s restarted the window


def test_route_order():
    names = ['a', 'b', 'c', 'd']
    assert nm.route_order(names, 'sequence') == names
    rng = random.Random(3)
    orders = {tuple(nm.route_order(names, 'shuffled', rng)) for _ in range(30)}
    assert len(orders) > 5 and all(sorted(o) == names for o in orders)
    for _ in range(30):
        assert nm.route_order(names, 'shuffled', rng, previous_last='a')[0] != 'a'


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
