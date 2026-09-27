"""
uwb_trilateration.py

Closed-form 3-anchor trilateration. No scipy dependency, no filtering or
smoothing (handled downstream in the Kalman filter stack).
"""

import math
from typing import Dict, Optional, Tuple

# ---- Configuration -------------------------------------------------------

# Anchor positions in the global UWB frame, meters. Update once anchors
# are surveyed and deployed.
ANCHOR_POSITIONS: Dict[int, Tuple[float, float, float]] = {
    1: (0.0, 0.0, 0.60),
    2: (10.0, 0.0, 0.630),
    3: (0.0, 15.0, 0.610),
}

# Whether the tag sits below the plane formed by the three anchors
# (e.g. anchors mounted high on enclosure fencing, tag on the ground
# robot). Determines which root of the z solution is physically valid.
TAG_BELOW_ANCHOR_PLANE = True

# ---------------------------------------------------------------------------


def calculate_position(
    ranges: Dict[int, float]
) -> Tuple[Optional[Tuple[float, float, float]], bool]:
    """
    Compute tag position (x, y, z) in the global UWB frame from three
    anchor-to-tag distances.

    ranges: {anchor_id: distance_m}. Must contain exactly the same three
    anchor IDs present in ANCHOR_POSITIONS (or a subset of it, as long
    as exactly three are supplied). Distances are expected in meters,
    so convert from cm before calling this if needed.

    Returns a (position, clamped) tuple:
      - position is None if fewer than three ranges are available, if
        the required anchors aren't in ANCHOR_POSITIONS, or if the
        anchor geometry itself is degenerate (anchors collinear).
      - clamped is True if a position was found but the ranges were
        geometrically inconsistent with the anchor layout (negative
        z^2, typically from noisy measurements), forcing z toward the
        anchor plane instead of its true solution. clamped is always
        False when position is None.
    """
    if len(ranges) != 3:
        return None, False

    anchor_ids = list(ranges.keys())
    try:
        p1 = ANCHOR_POSITIONS[anchor_ids[0]]
        p2 = ANCHOR_POSITIONS[anchor_ids[1]]
        p3 = ANCHOR_POSITIONS[anchor_ids[2]]
    except KeyError:
        return None, False

    r1 = ranges[anchor_ids[0]]
    r2 = ranges[anchor_ids[1]]
    r3 = ranges[anchor_ids[2]]

    p1 = _vec(p1)
    p2 = _vec(p2)
    p3 = _vec(p3)

    # Build a local orthonormal frame with p1 at the origin and p2 on
    # the local x-axis.
    ex = _sub(p2, p1)
    d = _norm(ex)
    if d == 0.0:
        return None, False
    ex = _scale(ex, 1.0 / d)

    p3_p1 = _sub(p3, p1)
    i = _dot(ex, p3_p1)
    ey_raw = _sub(p3_p1, _scale(ex, i))
    ey_norm = _norm(ey_raw)
    if ey_norm == 0.0:
        # p3 is collinear with p1 and p2, anchors are degenerate.
        return None, False
    ey = _scale(ey_raw, 1.0 / ey_norm)
    j = _dot(ey, p3_p1)

    ez = _cross(ex, ey)

    # Standard 3-sphere trilateration in the local frame.
    x = (r1 ** 2 - r2 ** 2 + d ** 2) / (2.0 * d)
    y = (r1 ** 2 - r3 ** 2 + i ** 2 + j ** 2 - 2.0 * i * x) / (2.0 * j)

    z_sq = r1 ** 2 - x ** 2 - y ** 2
    clamped = z_sq < 0.0
    if clamped:
        # Ranges are inconsistent with the anchor geometry (noise,
        # bad measurement, etc). Clamp instead of discarding outright.
        z_sq = 0.0
    z = math.sqrt(z_sq)
    if TAG_BELOW_ANCHOR_PLANE:
        z = -z

    # Convert local (x, y, z) back to the global frame.
    global_pos = _add(_add(p1, _scale(ex, x)), _add(_scale(ey, y), _scale(ez, z)))
    return global_pos, clamped


# ---- Minimal 3D vector helpers (no numpy/scipy) ---------------------------

def _vec(t: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return t


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    return math.sqrt(_dot(a, a))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )
