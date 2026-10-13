"""Unit tests for src/nodes/controllers/drive_command.py (run with pytest)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'nodes', 'controllers'))
from drive_command import DriveParams, command_line  # noqa: E402

P = DriveParams(r_eff=0.072, b_eff=0.22, p0=3.0, kp=1.8, max_percent=20.0, eps_v=0.02)


def parse(line):
    kind, left, right = line.split(',')
    return kind, float(left), float(right)


def test_stop():
    assert command_line(0.0, 0.0, P) == 'M,0,0'
    assert command_line(0.001, 0.01, P) == 'M,0,0'


def test_straight_forward_and_back():
    kind, left, right = parse(command_line(0.2, 0.0, P))
    assert kind == 'M' and left == right
    assert abs(left - (3.0 + 1.8 * 0.2 / 0.072)) < 0.06
    kind, left, right = parse(command_line(-0.2, 0.0, P))
    assert kind == 'M' and left == right < 0


def test_pivot_equal_magnitudes_and_direction():
    kind, left, right = parse(command_line(0.0, 0.6, P))
    assert kind == 'P' and left < 0 < right and abs(left + right) <= 1.0
    kind, left, right = parse(command_line(0.0, -0.6, P))
    assert kind == 'P' and right < 0 < left and abs(left + right) <= 1.0


def test_arc_never_opposite_signs():
    for v, w in ((0.05, 1.5), (0.05, -1.5), (-0.05, 1.5), (0.3, 0.6), (0.1, 3.0)):
        kind, left, right = parse(command_line(v, w, P))
        assert kind == 'M'
        assert left * right >= 0.0, (v, w, left, right)


def test_arc_inner_wheel_zero():
    kind, left, right = parse(command_line(0.05, 1.5, P))   # turning left, left wheel inner
    assert left == 0.0 and right > 0
    kind, left, right = parse(command_line(0.05, -1.5, P))
    assert right == 0.0 and left > 0


def test_turning_ratio():
    kind, left, right = parse(command_line(0.2, 0.3, P))
    assert right > left > 0


def test_clip():
    kind, left, right = parse(command_line(5.0, 0.0, P))
    assert left == right == 20.0
    kind, left, right = parse(command_line(0.0, 50.0, P))
    assert kind == 'P' and right == 20.0 and left == -20.0


def test_clip_wins_over_deadband():
    # Rule order: deadband (4) then clip (5). A limit below p0 still caps output.
    q = DriveParams(r_eff=0.072, b_eff=0.22, p0=12.0, kp=1.8, max_percent=10.0, eps_v=0.02)
    kind, left, right = parse(command_line(0.2, 0.0, q))
    assert left == right == 10.0
    q = DriveParams(r_eff=0.072, b_eff=0.22, p0=12.0, kp=1.8, max_percent=5.0, eps_v=0.02)
    kind, left, right = parse(command_line(0.2, 0.0, q))
    assert left == right == 5.0


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
