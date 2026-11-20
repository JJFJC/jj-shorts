"""Arm v2 CAD: closed-loop steppers + precision planetary gearboxes, on a linear-rail gantry.

    python v2/cad_v2.py  -> v2/out/stl/*.stl + v2/out/manifest.json (+ section halves in v2/out/cut/{keep,ghost})

Same kinematics and MuJoCo body frames as the servo arm (sim/fk.py), so the renders and IK still apply:
  turret      origin on the J1 axis at the platform top; J2 axis along Y at z = ARM["base_h"]
  upper_arm   origin on J2; J3 at z = ARM["upper"]
  forearm     origin on J3; J4 at z = ARM["fore"]
  wrist       origin on J4; roll axis along z at y = wrist_center_y()
  tool        coupler + gripper (reused from cad/out)
  x_carriage / z_carriage / world: gantry
Drive sizes come from v2/sizing.py (docs/ARM_V2.md). Motor/gearbox outlines are typical catalogue sizes.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(ROOT / "cad"), str(ROOT)]
import params as P  # noqa: E402
from internals import bearing_y, bearing_z, gear_z, ring_y, ring_z  # noqa: E402
from parts import box, cyl, fil, stadium, wrist_center_y  # noqa: E402
import internals as I  # noqa: E402

OUT = HERE / "out"
A = P.ARM
Z2 = A["base_h"]
YC = wrist_center_y()
PARTS: list[dict] = []


def add(name, body, role, shape, bom=None, note=""):
    PARTS.append(dict(name=name, body=body, role=role, shape=shape, bom=bom, note=note))


# ----------------------------------------------------------------------------- catalogue outlines


NEMA = {14: dict(sq=35.2, boss=22.0, bolt=26.0), 17: dict(sq=42.3, boss=22.0, bolt=31.0), 23: dict(sq=56.4, boss=38.1, bolt=47.1)}


def stepper_y(size, length, y_face, x=0.0, z=0.0, toward=+1, driver=0.0):
    """Stepper body along Y: mounting face at y_face, body extends away from `toward`.
    Returns (body, end caps, shaft). `driver` = closed-loop driver board thickness on the back."""
    n = NEMA[size]
    s = n["sq"]
    y0, y1 = sorted((y_face, y_face - toward * length))
    b = box(x - s / 2, x + s / 2, y0, y1, z - s / 2, z + s / 2)
    try:
        b = b.edges("|Y").chamfer(s * 0.09)
    except Exception:
        pass
    caps = box(x - s / 2 + 0.3, x + s / 2 - 0.3, y0, y0 + 7, z - s / 2 + 0.3, z + s / 2 - 0.3).union(
        box(x - s / 2 + 0.3, x + s / 2 - 0.3, y1 - 7, y1, z - s / 2 + 0.3, z + s / 2 - 0.3))
    try:
        caps = caps.edges("|Y").chamfer(s * 0.09)
    except Exception:
        pass
    b = b.cut(caps)
    if driver:
        yb = y_face - toward * length
        db = box(x - s / 2, x + s / 2, *sorted((yb, yb - toward * driver)), z - s / 2, z + s / 2)
        caps = caps.union(db)
    return b, caps


def stepper_z(size, length, z_face, x=0.0, y=0.0, toward=+1, driver=0.0):
    n = NEMA[size]
    s = n["sq"]
    z0, z1 = sorted((z_face, z_face - toward * length))
    b = box(x - s / 2, x + s / 2, y - s / 2, y + s / 2, z0, z1)
    try:
        b = b.edges("|Z").chamfer(s * 0.09)
    except Exception:
        pass
    caps = box(x - s / 2 + 0.3, x + s / 2 - 0.3, y - s / 2 + 0.3, y + s / 2 - 0.3, z0, z0 + 7).union(
        box(x - s / 2 + 0.3, x + s / 2 - 0.3, y - s / 2 + 0.3, y + s / 2 - 0.3, z1 - 7, z1))
    b = b.cut(caps)
    if driver:
        zb = z_face - toward * length
        caps = caps.union(box(x - s / 2, x + s / 2, y - s / 2, y + s / 2, *sorted((zb, zb - toward * driver))))
    return b, caps


def planetary_y(d_body, length, y_in, x=0.0, z=0.0, toward=+1, d_flange=None, sq_in=None):
    """Precision planetary along Y from its motor-side face y_in, output flange `toward`."""
    y1 = y_in + toward * length
    body = cyl(d_body / 2, (x, y_in, z), (x, y1, z))
    if sq_in:
        inp = box(x - sq_in / 2, x + sq_in / 2, *sorted((y_in, y_in + toward * 10)), z - sq_in / 2, z + sq_in / 2)
        body = body.union(inp)
    fl = cyl((d_flange or d_body) / 2, (x, y1, z), (x, y1 + toward * 6, z))
    for k in range(6):  # bolt holes on the output flange
        a = math.radians(30 + 60 * k)
        r = (d_flange or d_body) / 2 - 5
        fl = fl.cut(cyl(1.7, (x + r * math.cos(a), y1 - 1, z + r * math.sin(a)), (x + r * math.cos(a), y1 + toward * 8, z + r * math.sin(a))))
    return body, fl


# ----------------------------------------------------------------------------- shells


def shell_y(r, y0, y1, x=0.0, z=0.0, wall=2.6, fillet=6.0):
    s = fil(cyl(r, (x, y0, z), (x, y1, z)), fillet, "%CIRCLE")
    return s.cut(cyl(r - wall, (x, y0 + wall, z), (x, y1 - wall, z)))


def link_y(r1, r2, L, y0, y1, z0=0.0, wall=2.6, fillet=7.0):
    o = stadium(r1, r2, L, y0, y1, fillet).translate((0, 0, z0))
    i = stadium(r1 - wall, r2 - wall, L, y0 + wall, y1 - wall, 0).translate((0, 0, z0))
    return o.cut(i)


# ----------------------------------------------------------------------------- arm


def build_arm():
    off = P.PLATFORM["offset"]
    # ---- J1 (z_carriage frame, axis at (0, off)): fixed drum + bearing spigot, belt 20T -> 200T
    drum = fil(cyl(66, (0, off, 0), (0, off, 40)), 6, ">Z").cut(cyl(63.4, (0, off, 2.6), (0, off, 41)))
    add("j1_drum", "z_carriage", "shell", drum, ("printed PETG-CF", 1))
    sp = ring_z(30.0, 22.0, 2.6, 52.0, 0, off)
    add("j1_spigot", "z_carriage", "metal", sp, ("aluminium spigot (turned)", 1), "carries both 61812 bearings")
    bearing_z("j1_bearing_lo", "z_carriage", ("61812-2RS", 60, 78, 10), 8.0, 0, off)
    bearing_z("j1_bearing_hi", "z_carriage", ("61812-2RS", 60, 78, 10), 36.0, 0, off)
    m1, c1 = stepper_z(17, 48, -12.5, x=-74.0, y=off, toward=+1, driver=16)
    add("j1_motor", "z_carriage", "motor", m1, ("NEMA17-48 closed-loop (14-bit encoder driver)", 1))
    add("j1_motor_caps", "z_carriage", "motor_cap", c1)
    add("j1_motor_shaft", "z_carriage", "metal", cyl(2.5, (-74.0, off, -12.5), (-74.0, off, 52.0)), note="Ø5 shaft + extension through the platform")
    add("j1_pulley_20t", "z_carriage", "metal", gear_z(20, 2.0 / math.pi * 1.0, 44.0, 52.0, -74.0, off, bore=2.5), ("GT2 20T pulley", 1))
    # ---- turret (rotates): 200T belt ring, skirt, column, J2 housing with NEMA23 + 20:1
    add("j1_ring_200t", "turret", "metal", gear_z(200, 2.0 / math.pi, 44.0, 52.0, bore=56.0), ("GT2 200T pulley ring (or printed + belt-tooth insert)", 1))
    r_ring, r_pin, cx = 200 / math.pi, 20 / math.pi, -74.0
    ring_belt = ring_z(r_ring + 1.6, r_ring + 0.2, 44.5, 51.5).union(ring_z(r_pin + 1.6, r_pin + 0.2, 44.5, 51.5, cx, 0))
    for y0, y1 in ((r_pin, r_pin + 1.6), (-r_pin - 1.6, -r_pin)):  # short tangent runs between the pinion and the ring
        ring_belt = ring_belt.union(box(cx, -r_ring + 2, y0, y1, 44.5, 51.5))
    belt = ring_belt
    add("j1_belt", "turret", "rubber", belt, ("GT2 6 mm closed belt", 1))
    add("j1_turret_skirt", "turret", "shell", ring_z(42.0, 39.0, 2.0, 46.0), ("printed PETG-CF", 1))
    col = fil(cyl(54, (0, 0, 52), (0, 0, Z2 - 46.0)), 2, "<Z").cut(cyl(51.4, (0, 0, 51), (0, 0, Z2 - 46.0)))
    add("turret_column", "turret", "shell", col, ("printed PETG-CF", 1))
    # J2 housing: capsule along Y around NEMA23-76 (+ driver) + 2-stage 20:1 planetary, output toward +Y
    y_out = 48.0
    gb_len, m_len, drv = 72.0, 76.0, 18.0
    y_gb_in = y_out - 6 - gb_len
    y_m_face = y_gb_in
    hs = shell_y(47.0, y_m_face - m_len - drv - 6, y_gb_in + gb_len - 0.5, 0, Z2)
    hs = hs.cut(cyl(30.0, (0, -20, Z2 - 50), (0, -20, Z2 - 40)))  # opening into the column (cables)
    hs = hs.cut(cyl(31.5, (0, y_gb_in + gb_len - 4, Z2), (0, y_gb_in + gb_len + 1, Z2)))  # gearbox passes the end wall
    add("j2_housing", "turret", "shell", hs, ("printed PETG-CF, split on the XZ mid-plane", 1))
    y_end = y_m_face - m_len - drv - 6  # outer end face of the capsule; the grey cap sits on it (not coplanar with the wall)
    add("j2_housing_cap", "turret", "cap", fil(cyl(39.0, (0, y_end - 1.6, Z2), (0, y_end, Z2)), 0.8, "%CIRCLE"))
    mb, mc = stepper_y(23, m_len, y_m_face, 0, Z2, toward=+1, driver=drv)
    add("j2_motor", "turret", "motor", mb, ("NEMA23-76 closed-loop (57 mm driver on the back)", 1))
    add("j2_motor_caps", "turret", "motor_cap", mc)
    gb, fl = planetary_y(62, gb_len, y_gb_in, 0, Z2, toward=+1, d_flange=70, sq_in=60)
    add("j2_gearbox", "turret", "gearbox", gb, ("precision planetary 20:1, NEMA23 input, flange output, <= 5 arcmin", 1))
    add("j2_output_flange", "upper_arm", "metal", fl.translate((0, 0, -Z2)))
    add("upper_hub", "upper_arm", "cap", fil(cyl(42.0, (0, y_out, 0), (0, y_out + 6, 0)), 1.0, "%CIRCLE"), ("aluminium hub plate 6 mm", 1))
    # ---- upper arm: hub plate on the J2 flange, offset side link on +Y, J3 housing at the elbow
    L = A["upper"]
    ly0, ly1 = y_out + 6, y_out + 36
    add("upper_link", "upper_arm", "shell", link_y(48, 40, L, ly0, ly1).cut(cyl(36.0, (0, ly0 - 1, L), (0, ly1 + 1, L))), ("printed PETG-CF over an aluminium spine", 1))
    add("upper_spine", "upper_arm", "metal", box(-10, 10, ly0 + 5, ly0 + 25, 46, L - 40), ("aluminium 20x20 spine (or 20x20 tube)", 1),
        "carries the bending load; the printed shell is the cover")
    # J3: NEMA17-60 + 25:1 two-stage, output flange toward -Y (forearm side), housing from the forearm face to the link
    fy1 = -44.0  # forearm link inner face
    y3_out = fy1 + 8.0  # flange face
    g3_len, m3_len = 50.0, 60.0
    y3_gb = y3_out + 6 + g3_len
    h3 = shell_y(36.0, y3_out + 6 + 0.5, ly1, 0, L).cut(cyl(22.5, (0, y3_out + 5, L), (0, y3_out + 10, L)))
    add("j3_housing", "upper_arm", "shell", h3, ("printed PETG-CF", 1))
    m3b, m3c = stepper_y(17, m3_len, y3_gb, 0, L, toward=-1)
    add("j3_motor", "upper_arm", "motor", m3b, ("NEMA17-60 closed-loop (driver board remote, in the turret)", 1))
    add("j3_motor_caps", "upper_arm", "motor_cap", m3c)
    g3, f3 = planetary_y(44, g3_len, y3_gb, 0, L, toward=-1, d_flange=46, sq_in=44)
    add("j3_gearbox", "upper_arm", "gearbox", g3, ("precision planetary 25:1, NEMA17, flange output, <= 7 arcmin", 1))
    add("j3_output_flange", "forearm", "metal", f3.translate((0, 0, -L)))
    add("forearm_hub", "forearm", "cap", fil(cyl(36.0, (0, fy1, 0), (0, y3_out, 0)), 1.0, "%CIRCLE"), ("aluminium hub plate", 1))
    # ---- forearm: link on -Y from behind the elbow (J4 motor pod) to the wrist
    F = A["fore"]
    fy0 = fy1 - 30.0
    fl_ = link_y(40, 34, F + 72, fy0, fy1, z0=-72.0).cut(cyl(10.6, (0, fy1 - 3, F), (0, fy1 + 1, F)))
    add("forearm_link", "forearm", "shell", fl_, ("printed PETG-CF over an aluminium spine", 1))
    add("forearm_spine", "forearm", "metal", box(-9, 9, fy0 + 5, fy0 + 23, -50, F - 25), ("aluminium 20x20 spine", 1))
    # J4 drive pod behind the elbow: NEMA17-40 + 15:1, belt 1:1 to the wrist pulley inside the link
    zp = -72.0
    y4_face = fy1 + 2
    m4b, m4c = stepper_y(17, 40, y4_face + 46, 0, zp, toward=-1)
    g4, f4 = planetary_y(42, 40, y4_face + 46, 0, zp, toward=-1, d_flange=24, sq_in=42)
    add("j4_motor", "forearm", "motor", m4b, ("NEMA17-40 closed-loop", 1))
    add("j4_motor_caps", "forearm", "motor_cap", m4c)
    add("j4_gearbox", "forearm", "gearbox", g4, ("precision planetary 15:1, NEMA17, <= 7 arcmin", 1))
    pod = shell_y(33.0, y4_face - 2, y4_face + 46 + 40 + 4, 0, zp)
    add("j4_pod", "forearm", "shell", pod, ("printed PETG-CF", 1))
    y_belt = fy0 + 14
    pul = 2.0 * 30 / math.pi / 2  # GT2 30T pitch radius
    add("j4_pulley_motor", "forearm", "metal", I.gear_y(30, 2.0 / math.pi, y_belt - 4, y_belt + 4, 0, zp, bore=4.0), ("GT2 30T pulley", 1))
    add("j4_pulley_wrist", "forearm", "metal", I.gear_y(30, 2.0 / math.pi, y_belt - 4, y_belt + 4, 0, F, bore=4.0), ("GT2 30T pulley", 1))
    belt4 = box(pul, pul + 1.6, y_belt - 3, y_belt + 3, zp, F).union(box(-pul - 1.6, -pul, y_belt - 3, y_belt + 3, zp, F))
    add("j4_belt", "forearm", "rubber", belt4, ("GT2 6 mm closed belt + idler tensioner", 1))
    add("j4_shaft", "wrist", "metal", cyl(6.0, (0, y_belt - 4, 0), (0, YC - 30, 0)), ("steel shaft Ø12", 1))
    for i, yb in enumerate((fy1 - 12, fy1 - 4)):
        bearing_y(f"j4_bearing_{i + 1}", "forearm", ("6801-2RS", 12, 21, 5), yb, 0, F)
    # ---- wrist: puck + roll housing; J5 NEMA14 + 5:1 coaxial with the roll
    rw = 33.0
    outer = fil(cyl(rw, (0, YC - 30, 0), (0, YC + 30, 0)), 4, "%CIRCLE").union(fil(cyl(30, (0, YC, 0), (0, YC, A["wrist"] - 4)), 3, ">Z"))
    inner = cyl(rw - 2.6, (0, YC - 27.4, 0), (0, YC + 27.4, 0)).union(cyl(27.4, (0, YC, -1), (0, YC, A["wrist"] - 6.6)))
    wr = outer.cut(inner).cut(cyl(18.5, (0, YC, A["wrist"] - 11), (0, YC, A["wrist"]))).cut(cyl(6.5, (0, YC - 31, 0), (0, YC - 26, 0)))
    add("wrist_shell", "wrist", "shell", wr, ("printed PETG-CF, split on the XZ mid-plane", 1))
    m5b, m5c = stepper_z(14, 34, A["wrist"] - 36.0, 0, YC, toward=+1)
    add("j5_motor", "wrist", "motor", m5b, ("NEMA14-34 closed-loop", 1))
    add("j5_motor_caps", "wrist", "motor_cap", m5c)
    add("j5_gearbox", "wrist", "gearbox", cyl(18, (0, YC, A["wrist"] - 36.0), (0, YC, A["wrist"] - 10.0)), ("precision planetary 5:1, NEMA14", 1))
    bearing_z("j5_bearing", "wrist", ("6805-2RS", 25, 37, 7), A["wrist"] - 11.0, 0, YC)


# ----------------------------------------------------------------------------- linear-rail gantry


def rail(L, w, h, x0, x1, y, z, along="x"):
    if along == "x":
        r = box(x0, x1, y - w / 2, y + w / 2, z, z + h)
    else:
        r = box(y - w / 2, y + w / 2, x0, x1, z, z + h)  # unused
    return r


def build_gantry():
    X = P.X_TRAVEL
    xl = X / 2 + 120
    # X base: 10 mm aluminium plate on the bench + 2 HGR15 rails 80 mm apart
    add("x_base_plate", "world", "extrusion", box(-xl, xl, -70, 70, 0, 10), ("aluminium plate 10 mm (or a 4080 profile)", 1))
    for sy in (-40.0, 40.0):
        add(f"x_rail_{'a' if sy < 0 else 'b'}", "world", "metal", box(-xl + 10, xl - 10, sy - 7.5, sy + 7.5, 10, 25), ("HGR15 rail", 1))
    # X carriage (x_carriage frame: origin 20 above the table) on 4 HGR15 blocks
    for sx in (-60.0, 60.0):
        for sy in (-40.0, 40.0):
            add(f"x_block_{int(sx)}_{int(sy)}", "x_carriage", "block", box(sx - 30, sx + 30, sy - 17, sy + 17, 2, 18), ("HGR15CA block", 1))
    add("x_carriage_plate", "x_carriage", "extrusion", box(-80, 80, -70, 70, 18, 30), ("aluminium plate 12 mm", 1))
    # Z column: 4080 with the 80 side along Y, 2 MGN15 rails on its +Y face; T8x2 screw + motor on top
    zl = P.Z_COLUMN_LEN
    col = box(-20, 20, -70, 10, 30, 30 + zl)
    for sx in (-20, 20):
        for zz in range(0, 4):  # T-slot grooves (visual)
            pass
    col = col.cut(box(-3, 3, -71, 11, 31, 30 + zl - 1).intersect(box(-3, 3, -71, -65, 0, 2000)))
    add("z_column_4080", "x_carriage", "extrusion", col, ("4080 aluminium profile", 1))
    add("z_rail", "x_carriage", "metal", box(-7.5, 7.5, 10, 25, 40, 30 + zl - 10), ("HGR15 rail", 1))
    add("z_motor", "x_carriage", "motor", box(-21, 21, -51, -9, 30 + zl, 30 + zl + 48), ("NEMA17 for Z (T8x2 lead screw)", 1))
    add("z_screw", "x_carriage", "metal", cyl(4.0, (0, -30, 40), (0, -30, 30 + zl)), ("T8x2 lead screw", 1))
    add("x_motor_v2", "x_carriage", "motor", box(60, 102, -21, 21, -40, 8), ("NEMA17 for X (GT2 belt)", 1))
    # Z carriage (z_carriage frame: origin on the platform at the column face) on 4 MGN15H blocks
    for sz in (-100.0, -10.0):
        add(f"z_block_{int(-sz)}", "z_carriage", "block", box(-17, 17, -12, -2, sz - 30, sz + 30), ("HGR15CA block", 1))
    add("z_carriage_plate", "z_carriage", "extrusion", box(-45, 45, -2, 8, -140, 20), ("aluminium plate 10 mm", 1))
    off = P.PLATFORM["offset"]
    plat = box(-110, 80, 8, off + 70, -12, 0).union(box(-6, 6, 8, off - 30, -60, -12))  # platform + gusset
    plat = plat.cut(cyl(12.0, (-74.0, off, -13), (-74.0, off, 1)))  # J1 motor boss + shaft
    add("platform_v2", "z_carriage", "extrusion", plat, ("aluminium plate 12 mm + gusset", 1))


def main():
    build_arm()
    build_gantry()
    (OUT / "stl").mkdir(parents=True, exist_ok=True)
    man = []
    for p in PARTS:
        cq.exporters.export(p["shape"], str(OUT / "stl" / f"{p['name']}.stl"), tolerance=0.05, angularTolerance=0.12)
        v = p["shape"].val()
        man.append(dict(name=p["name"], body=p["body"], role=p["role"], bom=p["bom"], note=p["note"], valid=v.isValid()))
    (OUT / "manifest.json").write_text(json.dumps(man, indent=1))
    # section halves (body x = 0 plane) for the drivetrain view
    for sub in ("keep", "ghost"):
        (OUT / "cut" / sub).mkdir(parents=True, exist_ok=True)
    cman = []
    for p in PARTS:
        if p["body"] not in ("turret", "upper_arm", "forearm", "wrist") or p["role"] not in ("shell", "cap"):
            continue
        k, r = I.halves(p["shape"], "x")
        for sub, sh in (("keep", k), ("ghost", r)):
            if sh is not None:
                cq.exporters.export(sh, str(OUT / "cut" / sub / f"{p['name']}.stl"), tolerance=0.05, angularTolerance=0.12)
                cman.append(dict(name=p["name"], body=p["body"], role=p["role"], part=sub))
    (OUT / "cut" / "manifest.json").write_text(json.dumps(cman, indent=1))
    print(len(man), "parts; invalid:", [m["name"] for m in man if not m["valid"]], ";", len(cman), "section halves")


if __name__ == "__main__":
    main()
