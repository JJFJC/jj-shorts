"""Finishing tools for small parts: compliant rotary spindles (sand / fine sand / polish) and a
servo bench vise.

    python cad/finishing_tools.py  -> cad/out/finishing/*.stl (+ STEP for printed parts, manifest)

Spindle tool (tool frame: z along the tool axis from the J5 flange; the coupler tool half
occupies z 8..16):
  housing (printed) -> spring float carriage (2 steel guide rods, +-4 mm travel, hall sensor
  reads the deflection = contact force) -> 12 V brushless micro spindle -> ER11 collet ->
  standard 2.35 / 3.175 mm shank bit (sanding drum, felt bob, brush, burr).
A tiny MCU in the housing answers on the Feetech TTL bus as one more ID: goal speed in, float
deflection out. Power comes through the 2 new high-current coupler contacts.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_all import fits_bed  # noqa: E402
from parts import box, coupler_tool_half, cone, cyl, fil  # noqa: E402

OUT = HERE / "out" / "finishing"

SPINDLE = dict(housing_d=56.0, housing_top=52.0, motor_d=36.0, motor_len=55.0, nose_d=19.0, nose_len=15.0,
               shank_d=2.35, shank_len=22.0, float_mm=4.0, spring_rate_n_per_mm=1.2)
BITS = {
    "sand": dict(kind="drum", d=12.7, h=12.7, role="abrasive", note="sanding drum, 240 grit (layer lines)"),
    "fine": dict(kind="drum", d=12.7, h=12.7, role="abrasive_fine", note="sanding drum / flap wheel, 600 grit"),
    "polish": dict(kind="felt", d=15.0, h=16.0, role="felt", note="felt bob + plastic polishing compound"),
}
VISE = dict(x=0.0, y=205.0, base=(130.0, 80.0, 22.0), jaw_h=30.0, jaw_w=60.0)


def z_stack():
    s = SPINDLE
    z_motor = s["housing_top"] + 6.0
    z_nose = z_motor + s["motor_len"]
    z_shank = z_nose + s["nose_len"]
    z_bit = z_shank + s["shank_len"] - 6.0
    return z_motor, z_nose, z_shank, z_bit


def spindle_tool(bit_key: str):
    s = SPINDLE
    z_motor, z_nose, z_shank, z_bit = z_stack()
    r = s["housing_d"] / 2
    housing = fil(cyl(r, (0, 0, 16), (0, 0, s["housing_top"])), 3.0, ">Z").union(cone(r - 2, s["motor_d"] / 2 + 4, (0, 0, s["housing_top"]), (0, 0, z_motor + 4)))
    housing = housing.cut(cyl(s["motor_d"] / 2 + 1.5, (0, 0, s["housing_top"] - 22), (0, 0, z_motor + 5)))  # float cavity
    for k in range(6):  # cooling slots
        housing = housing.cut(box(-1.5, 1.5, -r - 1, -r + 6, 22 + k * 4.5, 24.5 + k * 4.5))
    rods = cyl(2.0, (-16, 0, 20), (-16, 0, z_motor + 2)).union(cyl(2.0, (16, 0, 20), (16, 0, z_motor + 2)))
    carriage = fil(box(-20, 20, -8, 8, s["housing_top"] - 18, s["housing_top"] - 10), 2.0, "|Z")
    motor = cyl(s["motor_d"] / 2, (0, 0, z_motor), (0, 0, z_nose))
    nose = cyl(s["nose_d"] / 2, (0, 0, z_nose), (0, 0, z_shank)).union(cone(8.0, 5.5, (0, 0, z_shank - 4), (0, 0, z_shank + 2)))
    shank = cyl(s["shank_d"] / 2, (0, 0, z_shank), (0, 0, z_shank + s["shank_len"]))
    b = BITS[bit_key]
    if b["kind"] == "drum":
        bit = cyl(b["d"] / 2, (0, 0, z_bit), (0, 0, z_bit + b["h"]))
    else:
        bit = cone(b["d"] / 2, 2.5, (0, 0, z_bit), (0, 0, z_bit + b["h"]))
    tcp = z_bit + b["h"] * 0.5
    # --- detail: engraved grit label, 3 cap screws on the shoulder, hex ER11 nut
    label = {"sand": "240", "fine": "600", "polish": "POLISH"}[bit_key]
    txt = cq.Workplane("XZ").text(label, 6.5 if bit_key != "polish" else 4.6, -0.6, halign="center", valign="center").translate((0, -r + 0.05, 34))
    housing = housing.cut(txt)
    import math as _m

    screws = None
    for k in range(3):
        a = _m.radians(90 + k * 120)
        sx, sy = (r - 5.0) * _m.cos(a), (r - 5.0) * _m.sin(a)
        head = cyl(2.7, (sx, sy, s["housing_top"] - 0.2), (sx, sy, s["housing_top"] + 1.6)).cut(
            cq.Workplane("XY").polygon(6, 2.4).extrude(1.5).translate((sx, sy, s["housing_top"] + 0.4)))
        screws = head if screws is None else screws.union(head)
    nut = cq.Workplane("XY").polygon(6, s["nose_d"] + 3).extrude(8.0).translate((0, 0, z_shank - 9.0)).edges("not |Z").chamfer(0.8)
    return dict(coupler=coupler_tool_half(0.0), housing=housing, carriage=carriage, rods=rods, motor=motor, nose=nose, nut=nut,
                screws=screws, shank=shank, bit=bit, tcp=tcp)


def servo_vise():
    x, y = VISE["x"], VISE["y"]
    bx, by, bz = VISE["base"]
    base = fil(box(x - bx / 2, x + bx / 2, y - by / 2, y + by / 2, 0, bz), 5, "|Z")
    fixed = fil(box(x - VISE["jaw_w"] / 2, x + VISE["jaw_w"] / 2, y + 4, y + 22, bz, bz + VISE["jaw_h"]), 2, "|Z")
    moving = fil(box(x - VISE["jaw_w"] / 2, x + VISE["jaw_w"] / 2, y - 26, y - 8, bz, bz + VISE["jaw_h"]), 2, "|Z")
    screw = cyl(4.0, (x, y - 60, bz + 10), (x, y + 4, bz + 10))
    servo = box(x - 12.5, x + 12.5, y - 82, y - 37, bz - 6, bz + 29)  # STS3215 turning the screw (bus ID 21)
    pads = box(x - 28, x + 28, y + 2.5, y + 4, bz + 6, bz + VISE["jaw_h"]).union(box(x - 28, x + 28, y - 8, y - 6.5, bz + 6, bz + VISE["jaw_h"]))
    part = fil(box(x - 15, x + 15, y - 6.5, y + 2.5, bz + 6, bz + 6 + 34), 2.5, "|Y")  # a 30 x 34 mm printed pendant
    part = part.cut(cyl(2.0, (x, y - 8, bz + 34), (x, y + 4, bz + 34)))
    bolts = None
    for sx_ in (-1, 1):
        for sy_ in (-1, 1):
            hx, hy = x + sx_ * (bx / 2 - 8), y + sy_ * (by / 2 - 8)
            b_ = cyl(3.6, (hx, hy, bz - 0.2), (hx, hy, bz + 2.4)).cut(cq.Workplane("XY").polygon(6, 3.0).extrude(2).translate((hx, hy, bz + 0.8)))
            bolts = b_ if bolts is None else bolts.union(b_)
    horn = cyl(10.0, (x, y - 37, bz + 10), (x, y - 33, bz + 10))  # servo horn coupling to the screw
    return dict(base=base, fixed=fixed, moving=moving, screw=screw, servo=servo, pads=pads, part=part, bolts=bolts, horn=horn)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    man = []
    roles = dict(coupler="shell", housing="shell", carriage="cap", rods="metal", motor="motor", nose="metal", nut="metal", screws="screw", shank="metal")
    printed = {"housing", "carriage"}
    for key in BITS:
        t = spindle_tool(key)
        for name, shape in t.items():
            if name == "tcp":
                continue
            role = roles.get(name, BITS[key]["role"])
            fname = f"spindle_{key}_{name}"
            cq.exporters.export(shape, str(OUT / f"{fname}.stl"), tolerance=0.03, angularTolerance=0.08)
            s = shape.val()
            bb = s.BoundingBox()
            man.append(dict(name=fname, role=role, body="tool", tool=key, printed=name in printed, valid=s.isValid(),
                            fits_bed=fits_bed([bb.xlen, bb.ylen, bb.zlen])[0]))
            if name in printed and key == "sand":
                cq.exporters.export(shape, str(OUT / f"spindle_{name}.step"))
        man[-1]["tcp_mm"] = t["tcp"]
    vroles = dict(base="shell", fixed="cap", moving="cap", screw="metal", servo="motor", pads="rubber", part="part", bolts="screw", horn="metal")
    for name, shape in servo_vise().items():
        cq.exporters.export(shape, str(OUT / f"vise_{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        s = shape.val()
        man.append(dict(name=f"vise_{name}", role=vroles[name], body="world", printed=name in ("base", "fixed", "moving"), valid=s.isValid()))
    (OUT / "manifest.json").write_text(json.dumps(man, indent=1))
    tcp = spindle_tool("sand")["tcp"]
    print(len(man), "parts; invalid:", [m["name"] for m in man if not m["valid"]], f"; spindle TCP {tcp:.0f} mm from the J5 flange")


if __name__ == "__main__":
    main()
