"""Gantry Arm candle cell: station layout (world frame of the Gantry Arm) + wide candle gripper.

    python cad/arm_cell.py  -> cad/out/arm_cell/*.stl (+ manifest.json)

World frame = Gantry Arm world: X along the rail (centre 0), +Y out over the work area, z up
(table at 0). Stations sit where the plugin IK says the arm can reach (see README table).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
from cell import H_MOLD, box, candle_solid, cyl, fil, mold_halves, piston  # noqa: E402

OUT = HERE / "out" / "arm_cell"

# ---------------------------------------------------------------- station positions (mm)
P1S = dict(w=389.0, d=389.0, h=458.0, front_x=-640.0, y=300.0)  # door faces +X (toward the rail)
STATIONS = {
    "flex_lip": (-600.0, 300.0, 75.0),  # curved lip at the printer mouth, bed height
    "cool_rack": (-170.0, 250.0, 0.0),
    "pour_scale": (60.0, 230.0, 60.0),
    "demold_post": (170.0, 210.0, 40.0),
    "sander": (320.0, 470.0, 150.0),  # drum top; candle held horizontally above it (IK: horizontal tool needs y 420-550 here)
    "output": (440.0, 200.0, 20.0),
}
GRIP = dict(open_max=95.0, jaw_len=80.0, body_z0=16.0, body_h=40.0)
TOOL_TCP = GRIP["body_z0"] + GRIP["body_h"] + GRIP["jaw_len"] * 0.55  # grip centre along the tool axis


def printer():
    x1 = P1S["front_x"]
    x0 = x1 - P1S["d"]
    y0, y1 = P1S["y"] - P1S["w"] / 2, P1S["y"] + P1S["w"] / 2
    shell = fil(box(x0, x1, y0, y1, 0, P1S["h"]), 12, "|X")
    shell = shell.cut(box(x1 - 330, x1 + 1, y0 + 30, y1 - 30, 40, P1S["h"] - 90))  # open door / chamber
    bed = box(x1 - 300, x1 - 30, P1S["y"] - 128, P1S["y"] + 128, 62, 70)
    tab = box(x1 - 30, x1 - 18, P1S["y"] - 20, P1S["y"] + 20, 68.5, 69.2)
    return shell, bed.union(tab)


def molds_on_plate():
    out = None
    for i, dy in enumerate((-70, 0, 70)):
        for h in mold_halves():
            m = h.translate((P1S["front_x"] - 120, P1S["y"] + dy, 70))
            out = m if out is None else out.union(m)
    return out


def flex_lip():
    x, y, z = STATIONS["flex_lip"]
    shelf = box(x - 40, x + 60, y - 140, y + 140, z - 8, z)
    lip = cyl(60, (x + 60, y - 140, z - 60), (x + 60, y + 140, z - 60)).intersect(box(x + 40, x + 130, y - 141, y + 141, z - 60, z + 1))
    legs = box(x - 30, x + 110, y - 150, y - 130, 0, z - 8).union(box(x - 30, x + 110, y + 130, y + 150, 0, z - 8))
    tray = fil(box(x + 70, x + 220, y - 150, y + 150, 0, 25), 4, "|Z").cut(box(x + 74, x + 216, y - 146, y + 146, 3, 30))
    return shelf.union(lip).union(legs), tray


def cool_rack():
    x, y, _ = STATIONS["cool_rack"]
    rack = None
    for z in (0, 130):
        shelf = box(x - 170, x + 170, y - 60, y + 60, z, z + 8)
        rack = shelf if rack is None else rack.union(shelf)
    for sx in (-1, 1):
        for sy in (-1, 1):
            rack = rack.union(box(x + sx * 165 - 5, x + sx * 165 + 5, y + sy * 55 - 5, y + sy * 55 + 5, 0, 138))
    return rack


def pour_station():
    x, y, z = STATIONS["pour_scale"]
    base = fil(box(x - 70, x + 70, y - 70, y + 70, 0, z - 12), 6, "|Z")
    scale = box(x - 55, x + 55, y - 55, y + 55, z - 12, z)
    post = cyl(14, (x, y + 150, 0), (x, y + 150, 330))
    pot = fil(cyl(80, (x, y + 150, 330), (x, y + 150, 470)), 6, "%CIRCLE")
    spout = cyl(6, (x, y + 75, 360), (x, y + 6, z + H_MOLD + 40)).union(cyl(4, (x, y + 6, z + H_MOLD + 40), (x, y, z + H_MOLD + 18)))
    valve = box(x - 12, x + 12, y + 30, y + 54, z + H_MOLD + 55, z + H_MOLD + 80)
    return base.union(scale), post, pot, spout, valve


def demold_post():
    x, y, z = STATIONS["demold_post"]
    base = fil(box(x - 55, x + 55, y - 55, y + 55, 0, z), 5, "|Z")
    pin = cyl(5, (x, y, z), (x, y, z + 92))  # mold is pressed down over it: piston + candle rise out of the top
    return base, pin


def sander():
    x, y, z = STATIONS["sander"]
    frame = fil(box(x - 70, x + 70, y - 110, y + 110, 0, z - 70), 6, "|Z")
    arm_l = box(x - 60, x - 50, y - 95, y + 95, z - 70, z - 25)
    arm_r = box(x + 50, x + 60, y - 95, y + 95, z - 70, z - 25)
    springs = cyl(6, (x - 55, y - 80, z - 95), (x - 55, y - 80, z - 70)).union(cyl(6, (x + 55, y - 80, z - 95), (x + 55, y - 80, z - 70)))
    drum = cyl(25, (x, y - 85, z - 25), (x, y + 85, z - 25))  # drum axis along Y = the held candle's axis
    motor = cyl(18, (x, y + 85, z - 25), (x, y + 140, z - 25))
    return frame.union(arm_l).union(arm_r), springs, drum, motor


def output_tray():
    x, y, z = STATIONS["output"]
    t = fil(box(x - 80, x + 80, y - 110, y + 110, 0, z), 5, "|Z").cut(box(x - 74, x + 74, y - 104, y + 104, 4, z + 1))
    return t


def wide_gripper(opening: float = 62.0):
    """Tool frame (z along the tool axis from the J5 flange, coupler tool half at z 8..16)."""
    z0, h = GRIP["body_z0"], GRIP["body_h"]
    body = fil(box(-70, 70, -22, 22, z0, z0 + h), 6, "|Z")
    body = body.union(fil(cyl(28, (0, 0, z0 - 0.5), (0, 0, z0 + 8)), 2, ">Z"))  # matches the coupler footprint
    rail = box(-66, 66, -6, 6, z0 + h, z0 + h + 6)
    jaws, pads = None, None
    for s in (-1, 1):
        xi = s * opening / 2
        j = fil(box(min(xi, xi + s * 14), max(xi, xi + s * 14), -20, 20, z0 + h, z0 + h + GRIP["jaw_len"]), 3, "|Z")
        j = j.cut(box(min(xi - s * 1, xi + s * 9), max(xi - s * 1, xi + s * 9), -9, 9, z0 + h + 10, z0 + h + GRIP["jaw_len"] + 1))  # V-ish notch
        p = box(min(xi, xi + s * 3), max(xi, xi + s * 3), -18, 18, z0 + h + 12, z0 + h + GRIP["jaw_len"] - 4)
        jaws = j if jaws is None else jaws.union(j)
        pads = p if pads is None else pads.union(p)
    return body.union(rail), jaws, pads


def build():
    parts = []

    def add(name, role, shape, body="world", printed=False):
        parts.append((name, role, shape, body, printed))

    sh, bed = printer()
    add("p1s_shell", "printer", sh)
    add("p1s_plate", "steel", bed)
    add("p1s_molds", "mold", molds_on_plate())
    lip, tray = flex_lip()
    add("flex_lip", "frame", lip, printed=True)
    add("mold_catch_tray", "frame", tray, printed=True)
    add("cool_rack", "frame", cool_rack())
    x, y, _ = STATIONS["cool_rack"]
    for i, (dx, zz) in enumerate(((-110, 8), (0, 8), (110, 8), (-110, 138), (0, 138), (110, 138))):
        for k, h in enumerate(mold_halves()):
            add(f"rack_mold_{i}{k}", "mold", h.translate((x + dx, y, zz)))
        add(f"rack_wax_{i}", "wax", candle_solid(0.97).translate((x + dx, y, zz)))
    base, post, pot, spout, valve = pour_station()
    add("pour_base", "frame", base)
    add("pour_post", "frame", post)
    add("wax_pot", "steel", pot)
    add("pour_spout", "steel", spout)
    add("pour_valve", "dark", valve)
    px, py, pz = STATIONS["pour_scale"]
    for k, h in enumerate(mold_halves()):
        add(f"pour_mold_{k}", "mold", h.translate((px, py, pz)))
    add("pour_piston", "mold2", piston().translate((px, py, pz)))
    add("pour_wax", "wax_liquid", candle_solid(0.6).translate((px, py, pz)))
    db, pin = demold_post()
    add("demold_base", "frame", db, printed=True)
    add("demold_pin", "steel", pin)
    fr, springs, drum, motor = sander()
    add("sander_frame", "frame", fr)
    add("sander_springs", "steel", springs)
    add("sander_drum", "abrasive", drum)
    add("sander_motor", "dark", motor)
    add("output_tray", "frame", output_tray(), printed=True)
    ox, oy, oz = STATIONS["output"]
    for i, (dx, dy) in enumerate(((-40, -55), (40, -55), (-40, 45))):
        add(f"out_candle_{i}", "wax", candle_solid(1.0).translate((ox + dx, oy + dy, oz - 6)))
    gb, jaws, pads = wide_gripper()
    add("candle_gripper_body", "shell", gb, body="tool", printed=True)
    add("candle_gripper_jaws", "cap", jaws, body="tool", printed=True)
    add("candle_gripper_pads", "rubber", pads, body="tool")
    # the candle being sanded, held coaxially in the jaws (tool frame)
    held = candle_solid(1.0).translate((0, 0, -6.0 - 80 / 2)).translate((0, 0, TOOL_TCP))
    add("held_candle", "wax", held, body="tool")
    return parts


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    man = []
    for name, role, shape, body, printed in build():
        s = shape.val() if shape.size() == 1 else shape.combine().val()
        cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.08, angularTolerance=0.15)
        bb = s.BoundingBox()
        man.append(dict(name=name, role=role, body=body, printed=printed, valid=s.isValid(), dims=[round(bb.xlen), round(bb.ylen), round(bb.zlen)]))
    (OUT / "manifest.json").write_text(json.dumps(man, indent=1))
    (OUT / "layout.json").write_text(json.dumps(dict(stations=STATIONS, tool_tcp=TOOL_TCP, p1s=P1S), indent=1))
    print(len(man), "parts; invalid:", [m["name"] for m in man if not m["valid"]], "; TOOL_TCP", TOOL_TCP)


if __name__ == "__main__":
    main()
