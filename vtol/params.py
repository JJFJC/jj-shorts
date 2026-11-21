"""KESTREL - original quad tilt-rotor VTOL: blended-wing body + front canard.

Units: mm, g, degrees. Frame: x aft (0 = main-wing leading edge at the body centreline... see
SECTIONS), y right, z up. Everything in cad/, sim/ and render/ reads this file.
"""

import math

# ---------------------------------------------------------------- blended main wing
# (y, chord, x_le, thickness) half-span sections; the body melts into the wing (lofted, smooth)
SECTIONS = [
    (0.0, 420.0, -120.0, 0.160),  # centre body: houses battery, FC, ESCs
    (70.0, 360.0, -80.0, 0.140),
    (130.0, 250.0, 0.0, 0.110),  # body -> wing blend
    (500.0, 140.0, 0.0, 0.090),  # tip (x_le set from sweep below)
]
SPAN = 1000.0
LE_SWEEP = 22.0  # outer wing
WASHOUT = 3.0
SECTIONS[-1] = (SECTIONS[-1][0], SECTIONS[-1][1], (SECTIONS[-1][0] - SECTIONS[-2][0]) * math.tan(math.radians(LE_SWEEP)), SECTIONS[-1][3])
WINGLET = dict(height=95.0, root_chord=130.0, tip_chord=70.0, sweep=35.0)

# ---------------------------------------------------------------- canard (second wing)
CANARD = dict(span=540.0, chord=115.0, x_le=-330.0, z=14.0, thickness=0.11, incidence=2.0)
NOSE = dict(x_tip=-390.0, x_join=-60.0, width=104.0, height=66.0)  # slim snout carrying the canard

# ---------------------------------------------------------------- quad tilt rotors
FRONT_ROTOR = dict(y=270.0, x_pivot=CANARD["x_le"] - 6.0, z=CANARD["z"])  # canard tips
# rear rotors: tilting PUSHERS behind the main-wing trailing edge (shaft up = hover, shaft aft = cruise)
REAR_ROTOR = dict(y=330.0, behind_te=20.0, z=0.0)
TILT_RANGE = (-8.0, 95.0)  # 0 = cruise (thrust forward), 90 = hover (thrust up)
MOTOR = dict(name="2207 1700KV", mass_g=33.0, max_thrust_g=1250.0)  # 4S, 7x4.5 (vendor-typical)
PROP_D_IN = 7.0


def chord_at(y: float) -> tuple[float, float, float]:
    """(chord, x_le, thickness) of the main wing at |y| by linear interpolation."""
    y = abs(y)
    for (y0, c0, x0, t0), (y1, c1, x1, t1) in zip(SECTIONS, SECTIONS[1:]):
        if y0 <= y <= y1:
            f = (y - y0) / (y1 - y0)
            return c0 + f * (c1 - c0), x0 + f * (x1 - x0), t0 + f * (t1 - t0)
    _, c, x, t = SECTIONS[-1]
    return c, x, t


# ---------------------------------------------------------------- mass budget (g)
MASS = {
    "blended wing + body (LW-PLA, 8 pieces)": 330,
    "nose snout + hatch (LW-PLA)": 45,
    "canard (LW-PLA, 2 pieces)": 55,
    "winglets (2)": 20,
    "nacelles + tilt pods (PETG, 4)": 95,
    "carbon spars (main 8 mm, canard 6 mm) + joiners": 40,
    "motors (4x 2207)": 132,
    "props (4x 7in 2-blade)": 32,
    "ESCs (4-in-1 45 A)": 18,
    "flight controller (H743) + GPS/compass": 40,
    "receiver + antennas": 6,
    "tilt servos (4x 12 g metal gear)": 52,
    "elevon servos (2x 9 g)": 18,
    "wiring, connectors, screws": 60,
    "battery 4S 2200 mAh LiPo": 240,
}
AIR_DENSITY = 1.225
CL_MAX = 1.0
