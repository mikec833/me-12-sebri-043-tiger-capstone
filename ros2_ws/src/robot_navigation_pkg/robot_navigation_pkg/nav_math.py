"""
Pure Python navigation math (no ROS), unit tested in test/test_nav_math.py.
"""

import math
import random
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def wrap(angle: float) -> float:
    """Angle wrapped to (-pi, pi]."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# ---- heading alignment (brief 4.1) --------------------------------------------

def fit_heading(points: Sequence[Point]) -> Optional[Tuple[float, float, float]]:
    """Least squares line through points.

    Returns (heading, length, rms): heading of the fitted direction (rad),
    signed from the first point towards the last; length of the points'
    extent along that line (m); RMS perpendicular residual (m). None for
    fewer than 3 points.
    """
    n = len(points)
    if n < 3:
        return None
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    syy = sum((p[1] - my) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)   # principal axis
    ux, uy = math.cos(theta), math.sin(theta)
    along = [(p[0] - mx) * ux + (p[1] - my) * uy for p in points]
    across = [-(p[0] - mx) * uy + (p[1] - my) * ux for p in points]
    # Sign from first to last: average the ends a little against noise.
    k = max(1, n // 10)
    if sum(along[-k:]) / k < sum(along[:k]) / k:
        theta += math.pi
    rms = math.sqrt(sum(a * a for a in across) / n)
    return wrap(theta), max(along) - min(along), rms


class HeadingAligner:
    """Drive straight, fit a line through UWB fixes, return the heading.

    Tick driven (call step() at the controller rate) so it never blocks.
    step() returns one of
        ('drive', v)              command v, omega 0
        ('stop',)                 command zero (settling or between tries)
        ('done', psi, length, rms)
        ('fail', reason)
    """

    def __init__(self, v_align=0.15, d_align=1.5, min_length=1.0, max_rms=0.15,
                 settle_s=0.8, attempts=2, timeout_factor=3.0):
        self.v_align, self.d_align = v_align, d_align
        self.min_length, self.max_rms = min_length, max_rms
        self.settle_s, self.attempts = settle_s, attempts
        self.timeout_s = timeout_factor * d_align / max(v_align, 1e-3) + 5.0
        self.reset()

    def reset(self):
        self._attempt = 0
        self._begin_attempt(None)

    def _begin_attempt(self, now):
        self._phase = 'drive'
        self._t0 = now
        self._t_settle = None
        self._travelled0 = None
        self._points: List[Point] = []

    def add_fix(self, x: float, y: float):
        if self._phase in ('drive', 'settle'):
            self._points.append((x, y))

    def step(self, now: float, travelled: float):
        """travelled: cumulative distance from wheel odometry (m)."""
        if self._t0 is None:
            self._t0 = now
        if self._travelled0 is None:
            self._travelled0 = travelled
        if self._phase == 'drive':
            far = travelled - self._travelled0 >= self.d_align
            late = now - self._t0 > self.timeout_s
            if not (far or late):
                return ('drive', self.v_align)
            self._phase, self._t_settle = 'settle', now
        if self._phase == 'settle':
            if now - self._t_settle < self.settle_s:
                return ('stop',)
            fit = fit_heading(self._points)
            if fit is not None and fit[1] > self.min_length and fit[2] < self.max_rms:
                self._phase = 'done'
                return ('done',) + fit
            self._attempt += 1
            if self._attempt >= self.attempts:
                self._phase = 'failed'
                why = 'too few UWB fixes' if fit is None else (
                    f'path {fit[1]:.2f} m (need > {self.min_length}), '
                    f'residual {fit[2]:.2f} m (need < {self.max_rms})')
                return ('fail', why)
            self._begin_attempt(now)
            return ('stop',)
        return ('stop',)


# ---- go to goal control law (brief section 5) ------------------------------------

def goal_errors(x, y, psi, gx, gy):
    """Distance rho and heading error e_psi to the goal."""
    dx, dy = gx - x, gy - y
    return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - psi)


def rotate_command(e_psi, k_omega, omega_max, omega_min):
    w = clip(k_omega * e_psi, -omega_max, omega_max)
    if w != 0.0 and abs(w) < omega_min:
        w = math.copysign(omega_min, w)
    return 0.0, w


def drive_command(rho, e_psi, k_rho, k_psi, v_min, v_max, omega_drive):
    v = clip(k_rho * rho, v_min, v_max) * max(math.cos(e_psi), 0.0)
    w = clip(k_psi * e_psi, -omega_drive, omega_drive)
    return v, w


def slew(current, target, rate, dt):
    step = rate * dt
    return current + clip(target - current, -step, step)


# ---- geofence ----------------------------------------------------------------------

def inside_polygon(x: float, y: float, polygon: Sequence[Point]) -> bool:
    """Even odd rule. An empty polygon means no fence."""
    if len(polygon) < 3:
        return True
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def pairs(flat: Sequence[float]) -> List[Point]:
    if len(flat) % 2:
        raise ValueError('expected x, y pairs')
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)]


# ---- progress and routes ------------------------------------------------------------

class ProgressMonitor:
    """No progress: rho not reduced by `distance` within `window` seconds."""

    def __init__(self, distance=0.2, window=15.0):
        self.distance, self.window = distance, window
        self.reset(None, None)

    def reset(self, now, rho):
        self._t, self._best = now, rho

    def stalled(self, now, rho) -> bool:
        if self._t is None:
            self.reset(now, rho)
            return False
        if rho <= self._best - self.distance:
            self.reset(now, rho)
            return False
        return now - self._t > self.window


def route_order(names: Sequence[str], mode: str, rng: Optional[random.Random] = None,
                previous_last: Optional[str] = None) -> List[str]:
    """sequence: as listed. shuffled: new random order, never starting at the
    point the previous session ended on (it would be a zero length leg)."""
    names = list(names)
    if mode == 'sequence':
        return names
    if mode != 'shuffled':
        raise ValueError(f"route mode must be 'sequence' or 'shuffled', got {mode}")
    rng = rng or random.Random()
    for _ in range(20):
        rng.shuffle(names)
        if len(names) < 2 or names[0] != previous_last:
            break
    return names
