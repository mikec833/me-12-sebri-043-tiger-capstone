// Drive maths for controller_v4_6.html -> xiao_manual_control.ino (v4.6).
//
// Reconstructed from the firmware's command parser; the original module was
// not in the repo. Wire format the XIAO accepts:
//   C,<accel ms>,<brake ms>,<max %>   integers; 100-4000, 100-4000, 10-100
//   M|B,<left %>,<right %>            -100..100, clamped again to max % on the XIAO
//   P,<left %>,<right %>              spin in place: signs must differ and
//                                     |left + right| <= 1, else the XIAO ignores it
// Positive % = forward for both wheels (the firmware maps +100 % to 2000 us).
//
// Shared curve rule (joystick and benchmark diagonals): the outer wheel runs
// at the commanded speed, the inner wheel at speed * (1 - curve * |turn|),
// where turn is the sideways share of the stick (sin of its angle from
// straight ahead). Curve 0 % drives straight; 100 % at full sideways stops
// the inner wheel. Stick right always slows the right wheel, forward or reverse.

const DEADZONE = 0.12;                         // fraction of joystick radius
const PIVOT_BAND = Math.sin(12 * Math.PI / 180); // within 12 deg of sideways = spin in place
const DIAGONAL_TURN = Math.SQRT1_2;            // benchmark diagonals = joystick at 45 deg

const clampInt = (value, lo, hi) => Math.min(hi, Math.max(lo, Math.round(Number(value) || lo)));
const percent = value => clampInt(value, 10, 100);

function curveMix(speed, forwardSign, turn, curve) {
  const inner = Math.round(speed * (1 - (curve / 100) * Math.abs(turn)));
  const left = turn > 0 ? speed : inner, right = turn > 0 ? inner : speed;
  return { left: forwardSign * left, right: forwardSign * right };
}

function pivot(speed, toRight) {
  return { left: toRight ? speed : -speed, right: toRight ? -speed : speed, directPivot: true };
}

const STOP = { left: 0, right: 0, directPivot: false };

// x, y: joystick position in [-1, 1], screen axes (y < 0 is forward).
export function proportionalTargets(x, y, maxOutput, curveStrength) {
  const max = percent(maxOutput), curve = clampInt(curveStrength, 0, 100);
  const radius = Math.min(1, Math.hypot(x, y));
  if (radius < DEADZONE) return { ...STOP, label: "Center · stopped" };
  const speed = Math.round(max * (radius - DEADZONE) / (1 - DEADZONE));
  if (speed < 1) return { ...STOP, label: "Center · stopped" };
  const forward = -y / radius, turn = x / radius;
  if (Math.abs(forward) < PIVOT_BAND)
    return { ...pivot(speed, turn > 0), label: `Spin ${turn > 0 ? "right" : "left"} · ${speed}%` };
  const sign = forward > 0 ? 1 : -1;
  const side = Math.abs(turn) < 0.05 ? "" : turn > 0 ? " right" : " left";
  return { ...curveMix(speed, sign, turn, curve), directPivot: false,
    label: `${sign > 0 ? "Forward" : "Reverse"}${side} · ${speed}%` };
}

// direction: forward | reverse | left | right | forward-left | forward-right |
// reverse-left | reverse-right | stop
export function benchmarkTargets(direction, maxOutput, curveStrength) {
  const max = percent(maxOutput), curve = clampInt(curveStrength, 0, 100);
  switch (direction) {
    case "forward": return { left: max, right: max, directPivot: false };
    case "reverse": return { left: -max, right: -max, directPivot: false };
    case "left": return pivot(max, false);
    case "right": return pivot(max, true);
    case "forward-left": return { ...curveMix(max, 1, -DIAGONAL_TURN, curve), directPivot: false };
    case "forward-right": return { ...curveMix(max, 1, DIAGONAL_TURN, curve), directPivot: false };
    case "reverse-left": return { ...curveMix(max, -1, -DIAGONAL_TURN, curve), directPivot: false };
    case "reverse-right": return { ...curveMix(max, -1, DIAGONAL_TURN, curve), directPivot: false };
    default: return { ...STOP };
  }
}

// Sent before Arm and whenever a ramp slider changes; the XIAO resets max to
// 10 % on every disarm, so this must precede each "A".
export function rampSettings(accelMs, brakeMs, maxOutput) {
  return `C,${clampInt(accelMs, 100, 4000)},${clampInt(brakeMs, 100, 4000)},${percent(maxOutput)}`;
}
