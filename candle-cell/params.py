"""Candle cell: rotary carousel that pours, cools, demolds and finishes soy/paraffin candles.
Units mm / g / degC / s. Frame: z up, carousel centre at the origin, operator at -Y.
"""

import math

# ---------------------------------------------------------------- candle + mold
CANDLE = dict(d=60.0, h=80.0, draft_deg=1.0)  # pillar; top (poured face) slightly wider
MOLD = dict(wall=7.0, floor=6.0, rim=4.0, slide=16.0, pin_d=4.0, tab_d=20.0, tab_depth=1.6, wick_d=2.4)
MOLD_PLASTIC = "PETG (softens ~80 C)"  # soy/paraffin poured at <= 70 C. Beeswax -> print in ASA.

# ---------------------------------------------------------------- carousel
N_STATIONS = 12
PITCH_R = 230.0  # mold centres on this radius
DISC = dict(d=600.0, t=12.0, z=120.0)  # 12 mm plywood or 3 mm aluminium disc, top at z
STATIONS = {  # index -> job; index 0 faces the operator (-Y), counting counter-clockwise
    0: "load wick",
    1: "pour",
    2: "cool", 3: "cool", 4: "cool", 5: "cool", 6: "cool",
    7: "top-off pour",
    8: "cool", 9: "cool",
    10: "demold + eject",
    11: "release spray / spare",
}
POT = dict(d=170.0, h=150.0, z=300.0, capacity_kg=2.0, set_c=65.0, cutoff_c=95.0)


def station_xy(k: int) -> tuple[float, float]:
    a = math.radians(-90.0 + k * 360.0 / N_STATIONS)
    return PITCH_R * math.cos(a), PITCH_R * math.sin(a)


# ---------------------------------------------------------------- process (soy pillar blend, 60x80 mm)
WAX_DENSITY = 0.9  # g/cm3 liquid ~0.86-0.9
POUR_C = (60.0, 70.0)
COOL_TO_DEMOLD_MIN = 120.0  # with fans; verify with a test pour (thermometer in the core)
TOP_OFF_AT_MIN = 60.0
