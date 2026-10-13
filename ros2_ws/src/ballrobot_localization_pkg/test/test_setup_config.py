from ballrobot_localization_pkg import geometry as g
from ballrobot_localization_pkg import setup_config as sc


def test_shared_setups_resolve_and_give_anchors():
    for name in ('default', 'outdoor'):
        flat = sc.anchor_positions(sc.load(sc.resolve(name)))
        assert flat is not None and len(flat) == 9
        plane = g.plane_from_points(tuple(flat[0:3]), tuple(flat[3:6]), tuple(flat[6:9]))
        assert plane is not None


def test_outdoor_plane_height_matters():
    # A clamped solve sits on the outdoor plane (z = 1.345). Against the
    # default 1.3 m plane it would be 4.5 cm away and slip past the 1 mm gate.
    outdoor = sc.anchor_positions(sc.load(sc.resolve('outdoor')))
    default = sc.anchor_positions(sc.load(sc.resolve('default')))
    p_out = g.plane_from_points(tuple(outdoor[0:3]), tuple(outdoor[3:6]), tuple(outdoor[6:9]))
    p_def = g.plane_from_points(tuple(default[0:3]), tuple(default[3:6]), tuple(default[6:9]))
    clamped = (7.2, 3.1, 1.345)
    assert g.distance_to_plane(clamped, p_out) < 1e-3
    assert g.distance_to_plane(clamped, p_def) > 1e-3


def test_unknown_setup_lists_available():
    try:
        sc.resolve('no_such_setup')
        assert False
    except FileNotFoundError as exc:
        assert 'outdoor' in str(exc)


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
