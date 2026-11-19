"""VTOL tri-tiltrotor flying wing: every dimension, mass and component choice.

Units: mm, g, degrees. Frame: x aft from the root leading edge, y right wing, z up.
"""

import math

# ---------------------------------------------------------------- wing (trapezoid, swept)
SPAN = 1000.0
POD_HALF_W = 40.0  # wing panels start here (centre pod is 80 wide)
ROOT_CHORD = 230.0  # at the pod side; fits the 256 mm bed (diagonal) when printed upright
TIP_CHORD = 150.0
LE_SWEEP = 25.0
WASHOUT = 3.0  # tip twisted nose-down for flying-wing stability
THICKNESS = 0.10  # NACA 0010 (symmetric; reflex-free, stability from sweep + washout)
PANELS_PER_SIDE = 2  # printed segments per half wing
SPAR_D = 8.0  # carbon tube along the 25 % chord line
ELEVON = dict(y0=230.0, y1=480.0, chord_frac=0.25)

HALF = SPAN / 2


def chord(y: float) -> float:
    t = (abs(y) - POD_HALF_W) / (HALF - POD_HALF_W)
    t = min(max(t, 0.0), 1.0)
    return ROOT_CHORD + (TIP_CHORD - ROOT_CHORD) * t


def x_le(y: float) -> float:
    return max(abs(y) - POD_HALF_W, 0.0) * math.tan(math.radians(LE_SWEEP))


def twist(y: float) -> float:
    t = (abs(y) - POD_HALF_W) / (HALF - POD_HALF_W)
    return -WASHOUT * min(max(t, 0.0), 1.0)


# ---------------------------------------------------------------- fuselage pod
POD = dict(x_nose=-95.0, x_tail=300.0, width=80.0, height=62.0)

# ---------------------------------------------------------------- propulsion (tri-tilt)
FRONT_Y = 200.0  # tilt nacelles at +-200 mm span
PIVOT_AHEAD_OF_LE = 22.0  # pivot axis this far ahead of the local leading edge
TILT_RANGE = (-10.0, 95.0)  # deg: 0 = cruise (thrust forward), 90 = hover (thrust up); extra for yaw vectoring
REAR_Z = 45.0  # rear lift motor height above the boom
MOTOR = dict(name="2207 1700KV", mass_g=33.0, max_thrust_g=1250.0)  # 4S, 7x4.5 2-blade (vendor-typical; verify on chart)
PROP_D_FRONT_IN = 7.0
PROP_D_REAR_IN = 6.0

# ---------------------------------------------------------------- mass budget (g)
MASS = {
    "wing panels (LW-PLA, 4)": 230,
    "centre pod + hatch (LW-PLA/PETG)": 95,
    "nacelles + tilt pods (PETG)": 55,
    "rear boom + motor mount + fins": 45,
    "carbon spars + joiner": 30,
    "motors (3x 2207)": 99,
    "props (2x 7in, 1x 6in)": 24,
    "ESCs (3x 30 A BLHeli_S)": 30,
    "flight controller (H743 wing) + GPS/compass": 40,
    "receiver + antennas": 6,
    "tilt servos (2x 12 g metal gear)": 26,
    "elevon servos (2x 9 g)": 18,
    "wiring, connectors, screws": 45,
    "battery 4S 2200 mAh LiPo": 240,
}
AIR_DENSITY = 1.225
CL_MAX = 1.0  # flying wing with elevons trimmed (conservative)
