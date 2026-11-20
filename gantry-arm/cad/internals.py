"""Inside the arm: servos, gears, bearings, shafts, couplings, mounts and the cable path.

    python cad/internals.py   -> cad/out/internals/*.stl + manifest.json   (needs cad/out/servo_layout.json)
                                 cad/out/cutaway/*.stl  (printed shells cut on the x = 0 plane)

Every part is placed in the MuJoCo body frame it moves with (same convention as cad/parts.py):
  z_carriage  J1 fixed spigot + bearings + lower Oldham hub
  turret      J1 servo (inverted) + J2 twin servos, pinions, horns, bearings, mount plate
  upper_arm   J2 hub gear + hollow shaft (on its own origin = J2 axis); J3 hub gear + shaft (at z = upper)
  forearm     J3 twin servos, pinions, horns, bearings, mount plate (origin = J3 axis)
  wrist       J4 servo, J4 bearing, J5 micro servo + 1:1 gears + roll shaft + bearing, 6 V buck
Design notes and numbers: docs/MECHANICS.md.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import params as P  # noqa: E402
from parts import WALL, box, cyl, elbow_width, wrist_center_y  # noqa: E402

OUT = HERE / "out" / "internals"
CUT = HERE / "out" / "cutaway"
A, S = P.ARM, P.STYLE
L15, W15, H15 = P.STS3215.body
SH = P.SERVO_SHAFT_FROM_END
Z2 = A["base_h"]  # J2 axis height in the turret frame

PARTS: list[dict] = []


def add(name, body, role, shape, *, bom=None, joint="", note=""):
    PARTS.append(dict(name=name, body=body, role=role, shape=shape, bom=bom, joint=joint, note=note))


# ----------------------------------------------------------------------------- primitives along Y


def ring_y(r_out, r_in, y0, y1, x=0.0, z=0.0):
    s = cyl(r_out, (x, y0, z), (x, y1, z))
    return s.cut(cyl(r_in, (x, y0 - 1, z), (x, y1 + 1, z))) if r_in > 0 else s


def ring_z(r_out, r_in, z0, z1, x=0.0, y=0.0):
    s = cyl(r_out, (x, y, z0), (x, y, z1))
    return s.cut(cyl(r_in, (x, y, z0 - 1), (x, y, z1 + 1))) if r_in > 0 else s


def gear_profile(z: int, m: float, phase: float = 0.0, flip: bool = False):
    """Simple spur-gear outline (straight-flank teeth; visual + mass, not for cutting).
    Teeth centred at phase + 2*pi*i/z. flip=True mirrors v so the XY->XZ turn keeps angles in the x-z plane."""
    ra, rf, rp = m * (z / 2 + 1.0), m * (z / 2 - 1.25), m * z / 2
    pts = []
    sgn = -1.0 if flip else 1.0
    for i in range(z):
        a0 = 2 * math.pi * i / z + phase
        t = math.pi / z  # tooth angle at pitch circle
        for ang, r in ((a0 - 0.55 * t, rf), (a0 - 0.30 * t, ra), (a0 + 0.30 * t, ra), (a0 + 0.55 * t, rf)):
            pts.append((r * math.cos(ang), sgn * r * math.sin(ang)))
    return pts, rp


def mesh_phase(theta, z_hub, z_pin, hub_phase=0.0):
    """Pinion phase that meshes with the hub when the pinion sits at angle theta from the hub centre."""
    pitch = 2 * math.pi / z_hub
    e = (theta - hub_phase) - round((theta - hub_phase) / pitch) * pitch
    return theta + math.pi + e * z_hub / z_pin + math.pi / z_pin


def gear_y(z: int, m: float, y0: float, y1: float, x: float = 0.0, zc: float = 0.0, bore: float = 0.0, phase: float = 0.0):
    """Spur gear with its axis along Y, centred on (x, zc). 2D profile drawn in XY then turned to XZ."""
    pts, _ = gear_profile(z, m, phase, flip=True)
    g = cq.Workplane("XY").polyline(pts).close().extrude(y1 - y0)
    if bore:
        g = g.cut(cq.Workplane("XY").circle(bore).extrude(y1 - y0))
    # XY profile -> XZ plane: rotate -90 deg about X maps (u, v, w) -> (u, w, -v)
    g = g.rotate((0, 0, 0), (1, 0, 0), -90).translate((x, y0, zc))
    return g


def gear_z(z: int, m: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0, bore: float = 0.0, phase: float = 0.0):
    pts, _ = gear_profile(z, m, phase)
    g = cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0)
    if bore:
        g = g.cut(cq.Workplane("XY").circle(bore).extrude(z1 - z0))
    return g.translate((x, y, z0))


def bearing_y(name, body, des, y0, x=0.0, z=0.0, joint=""):
    d, D, w = des[1], des[2], des[3]
    t = (D - d) / 2
    add(f"{name}_races", body, "metal", ring_y(D / 2, D / 2 - 0.3 * t, y0, y0 + w, x, z).union(ring_y(d / 2 + 0.3 * t, d / 2, y0, y0 + w, x, z)),
        bom=(des[0], 1), joint=joint)
    add(f"{name}_seal", body, "rubber", ring_y(D / 2 - 0.3 * t, d / 2 + 0.3 * t, y0 + 0.4, y0 + w - 0.4, x, z), joint=joint)


def bearing_z(name, body, des, z0, x=0.0, y=0.0, joint=""):
    d, D, w = des[1], des[2], des[3]
    t = (D - d) / 2
    add(f"{name}_races", body, "metal", ring_z(D / 2, D / 2 - 0.3 * t, z0, z0 + w, x, y).union(ring_z(d / 2 + 0.3 * t, d / 2, z0, z0 + w, x, y)),
        bom=(des[0], 1), joint=joint)
    add(f"{name}_seal", body, "rubber", ring_z(D / 2 - 0.3 * t, d / 2 + 0.3 * t, z0 + 0.4, z0 + w - 0.4, x, y), joint=joint)


def sts_case(L, W, H, S_end):
    """Servo case in local coords: output shaft on +Z at the origin, body extends along +X (L - S_end)
    and -X (S_end), Z from -H to 0 (case), spline boss 0..5."""
    c = box(-S_end, L - S_end, -W / 2, W / 2, -H, 0)
    try:
        c = c.edges("|Z").fillet(2.0)
    except Exception:
        pass
    label = box(-S_end + 4, L - S_end - 4, -W / 2 - 0.2, -W / 2 + 0.3, -H + 6, -6)  # sticker recess (visual)
    spline = cyl(4.5, (0, 0, 0), (0, 0, 5))
    return c.cut(label), spline


def place_servo_y(case, spline, px, pz, phi_deg, y_top):
    """Shaft along +Y at (px, y_top, pz); the body's long axis at phi (degrees, in the x-z plane, from +x toward +z)."""
    out = []
    for s in (case, spline):
        s = s.rotate((0, 0, 0), (1, 0, 0), -90)  # local +Z -> +Y ; local +Y -> -Z
        s = s.rotate((0, 0, 0), (0, 1, 0), -phi_deg)  # +x toward +z by phi
        out.append(s.translate((px, y_top, pz)))
    return out


def horn_y(px, pz, y0, y1):
    h = cyl(10.0, (px, y0, pz), (px, y1, pz))
    for k in range(4):
        a = math.radians(45 + 90 * k)
        h = h.cut(cyl(1.3, (px + 7 * math.cos(a), y0 - 1, pz + 7 * math.sin(a)), (px + 7 * math.cos(a), y1 + 1, pz + 7 * math.sin(a))))
    return h


def cable(points, r=2.4):
    """Tube through straight segments with ball joints (a harness, visual)."""
    s = None
    for p0, p1 in zip(points[:-1], points[1:]):
        seg = cyl(r, p0, p1).union(cq.Workplane("XY").add(cq.Solid.makeSphere(r, cq.Vector(*p1))))
        s = seg if s is None else s.union(seg)
    return s


def coil_y(R, r, y, x=0.0, z=0.0, turns_deg=300):
    """Service loop: a torus arc around the Y axis."""
    t = cq.Solid.makeTorus(R, r, cq.Vector(x, y, z), cq.Vector(0, 1, 0))
    keep = cq.Workplane("XY").add(t)
    if turns_deg < 360:  # open the loop where the cable enters/leaves
        keep = keep.cut(box(x - 3, x + 3, y - r - 1, y + r + 1, z - R - r - 1, z))
    return keep


# ----------------------------------------------------------------------------- J1 base yaw


def build_j1():
    off = P.PLATFORM["offset"]
    b_lo, b_hi = P.BEARINGS["base_yaw"]
    # fixed spigot (printed PETG) in the z_carriage frame, J1 axis at (0, off)
    sp = ring_z(30.0, 22.0, 2.4, 40.0, 0, off).union(ring_z(36.0, 6.0, 2.4, 5.4, 0, off))
    sp = sp.union(cyl(30.0, (0, off, 37.0), (0, off, 40.0)))
    sp = sp.cut(cyl(6.0, (16.0, off, 30), (16.0, off, 41)))  # cable hole in the top plate
    add("j1_spigot", "z_carriage", "cap", sp, bom=("printed PETG", 1), joint="J1", note="fixed; carries both 61812 bearings")
    bearing_z("j1_bearing_lo", "z_carriage", b_lo, 6.0, 0, off, "J1")
    bearing_z("j1_bearing_hi", "z_carriage", b_hi, 29.0, 0, off, "J1")
    jaw = box(-12, 12, off - 3, off + 3, 43.0, 44.0)
    add("j1_oldham_lower", "z_carriage", "metal", cyl(12.0, (0, off, 40.0), (0, off, 43.0)).union(jaw), bom=("Oldham coupling 3-part, Ø25", 1), joint="J1")
    # turret side (rotates): skirt with the bearing seats, Oldham middle + upper, servo, cradle
    add("j1_turret_skirt", "turret", "cap", ring_z(42.0, 39.0, 3.0, 34.5), bom=("printed PETG", 1), joint="J1", note="outer race seat of the lower 61812; the collar bore seats the upper one")
    add("j1_oldham_slider", "turret", "accent_white", cyl(12.0, (0, 0, 43.0), (0, 0, 46.0)), joint="J1", note="POM disc")
    add("j1_oldham_upper", "turret", "metal", cyl(12.0, (0, 0, 46.0), (0, 0, 49.0)).union(box(-3, 3, -12, 12, 45.0, 46.0)), joint="J1")
    add("j1_horn", "turret", "metal", cyl(10.0, (0, 0, 49.0), (0, 0, 53.0)), joint="J1")
    case, spline = sts_case(L15, W15, H15, SH)
    # inverted: shaft points down (-z); body long axis along +x
    case = case.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, 58.0))
    spline = spline.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, 58.0))
    add("j1_servo", "turret", "servo", case, bom=("Feetech STS3215 12 V", 1), joint="J1")
    add("j1_servo_spline", "turret", "metal", spline, joint="J1")
    cx = (L15 - SH - SH) / 2  # case centre along x
    cr = box(cx - L15 / 2 - 3, cx + L15 / 2 + 3, -W15 / 2 - 3, W15 / 2 + 3, 72.0, 88.0).cut(box(cx - L15 / 2 - 0.3, cx + L15 / 2 + 0.3, -W15 / 2 - 0.3, W15 / 2 + 0.3, 70, 90))
    for sx in (-1, 1):  # two ribs to the column wall
        cr = cr.union(box(-3, 3, sx * (W15 / 2 + 2), sx * 43.0, 74.0, 86.0) if sx > 0 else box(-3, 3, -43.0, -(W15 / 2 + 2), 74.0, 86.0))
    add("j1_servo_cradle", "turret", "cap", cr, bom=("printed PETG", 1), joint="J1", note="M2 screws into the case side holes")


# ----------------------------------------------------------------------------- J2 / J3 geared pairs


def build_pair(joint, key, body_stator, z_axis, rotor_body, rotor_z, R_puck, layout):
    g = P.GEARS[joint]
    m, (zA, zB), zh = g["module"], g["z_pinion"], g["z_hub"]
    face = g["face"]
    bdes = P.BEARINGS[joint]
    shaft_r = bdes[0][1] / 2
    y_gear0, y_gear1 = 20.0, 20.0 + face
    y_horn0, y_horn1 = 16.0, 20.0
    y_spl = 11.0  # case top (spline boss 11..16)
    # rotor: hub gear + hollow shaft (wires inside)
    add(f"{key}_hub_gear", rotor_body, "steel_gear", gear_y(zh, m, y_gear0, y_gear1, 0, rotor_z, bore=shaft_r),
        bom=(f"steel spur gear m{m:g} {zh}T bore {2 * shaft_r:.0f}", 1), joint=key.upper())
    add(f"{key}_shaft", rotor_body, "metal", ring_y(shaft_r, shaft_r - 2.5 if shaft_r >= 10 else shaft_r - 2.0, 17.0, 62.0, 0, rotor_z),
        bom=(f"aluminium tube Ø{2 * shaft_r:.0f} x {2 * (shaft_r - (2.5 if shaft_r >= 10 else 2.0)):.0f}, 45 mm", 1), joint=key.upper(),
        note="clamped in the link hub with 2x M3 set screws on a flat; the harness runs inside")
    # stator: bearings in a boss on the +Y wall, pinions, horns, servos, mount plate
    y_wall = R_puck["y_wall_in"]
    w = bdes[0][3]
    yb2 = y_wall - w
    yb1 = yb2 - w
    for i, yb in enumerate((yb1, yb2)):
        bearing_y(f"{key}_bearing_{i + 1}", body_stator, bdes[i], yb, 0, z_axis, key.upper())
    boss = ring_y(bdes[0][2] / 2 + 4.0, bdes[0][2] / 2, yb1 - 1.0, y_wall, 0, z_axis)
    add(f"{key}_bearing_boss", body_stator, "cap", boss, bom=("printed PETG (glued into the puck)", 1), joint=key.upper())
    case, spline = sts_case(L15, W15, H15, SH)
    for tag, zp in (("A", zA), ("B", zB)):
        px, pz, phi = layout[tag]
        pz += z_axis
        ph = mesh_phase(math.atan2(pz - z_axis, px), zh, zp)
        add(f"{key}_pinion_{tag}", body_stator, "steel_gear", gear_y(zp, m, y_gear0, y_gear1, px, pz, bore=2.5, phase=ph),
            bom=(f"steel spur gear m{m:g} {zp}T, bolted to the servo horn", 1), joint=key.upper())
        add(f"{key}_horn_{tag}", body_stator, "metal", horn_y(px, pz, y_horn0, y_horn1), bom=("STS aluminium horn (with the servo)", 0), joint=key.upper())
        c, sp = place_servo_y(case, spline, px, pz, phi, y_spl + 5.0)
        add(f"{key}_servo_{tag}", body_stator, "servo", c.translate((0, -5.0, 0)), bom=("Feetech STS3250 12 V", 1), joint=key.upper())
        add(f"{key}_servo_spline_{tag}", body_stator, "metal", sp.translate((0, -5.0, 0)), joint=key.upper())
    # mount plate behind the cases (printed), screwed to standoffs moulded in the puck
    yp = y_spl - H15
    plate = ring_y(R_puck["r_in"] - 0.5, 0.0, yp - 3.0, yp, 0, z_axis)
    plate = plate.intersect(box(-100, 100, yp - 4, yp + 1, z_axis + R_puck.get("z_floor", -100), z_axis + 100))
    plate = plate.cut(cyl(6.0, (18.0, yp - 4, z_axis - 20.0), (18.0, yp + 1, z_axis - 20.0)))  # cable pass
    add(f"{key}_mount_plate", body_stator, "cap", plate, bom=("printed PETG", 1), joint=key.upper(), note="4x M2 into each case's back holes")
    # harness: service loop in the 11..16 slab around the shaft end, then into the hollow shaft
    loop = coil_y(13.0 if shaft_r >= 10 else 11.5, 2.2, 13.5, 0, z_axis)
    add(f"{key}_cable_loop", body_stator, "cable", loop, joint=key.upper(), note="±range/2 coiled loop; no slip ring")


def build_j2_j3():
    lay = json.loads((HERE / "out" / "servo_layout.json").read_text())
    rs, ws = S["shoulder_puck"]
    build_pair("shoulder_lift", "j2", "turret", Z2, "upper_arm", 0.0,
               dict(y_wall_in=ws / 2 - WALL, r_in=rs - WALL, z_floor=-26.0), lay["J2"][1])
    re_ = S["elbow_puck_r"]
    ey1 = 48.0
    build_pair("elbow_flex", "j3", "forearm", 0.0, "forearm", 0.0,
               dict(y_wall_in=ey1 - WALL, r_in=re_ - WALL), lay["J3"][1])
    # J1 -> J2 harness (turret): up through the spigot hole, a loop around the Oldham stack, past the J1 servo, into the puck
    h = cable([(16.0, 0.0, 41.0), (16.0, -24.0, 47.0), (28.0, -28.0, 62.0), (12.0, -36.0, 84.0), (-8.0, -34.0, 96.0),
               (18.0, -31.0, Z2 - 20.0), (18.0, -25.5, Z2 - 20.0), (18.0, 0.0, Z2 - 18.0), (14.0, 12.5, Z2 - 16.0), (0.0, 13.5, Z2 - 13.0)])
    add("j1_j2_harness", "turret", "cable", h, joint="J1", note="3-wire bus + 12 V + LED data, ~6 mm bundle")


# ----------------------------------------------------------------------------- wrist J4 + J5


def build_wrist():
    yc = wrist_center_y()
    rw, ww = S["wrist_puck"]
    # J4: STS3215 on the axis, horn toward the forearm (-Y), body pointing at the tool (+z).
    # Stack from the -Y wall: horn (3) | spline (5) | case (35) -> ends at y = 9; J5 starts at y ~10.
    y_w = yc - ww / 2  # outer -Y face of the wrist puck
    y_horn0, y_horn1 = y_w, y_w + 3.0
    case, spline = sts_case(L15, W15, H15, SH)
    c, sp = place_servo_y(case, spline, 0.0, 0.0, 90.0, 0.0)
    c = c.rotate((0, 0, 0), (0, 0, 1), 180).translate((0, y_horn1 + 5.0, 0))  # shaft now along -Y
    sp = sp.rotate((0, 0, 0), (0, 0, 1), 180).translate((0, y_horn1 + 5.0, 0))
    add("j4_servo", "wrist", "servo", c, bom=("Feetech STS3215 12 V", 1), joint="J4")
    add("j4_servo_spline", "wrist", "metal", sp, joint="J4")
    add("j4_horn", "wrist", "metal", horn_y(0.0, 0.0, y_horn0, y_horn1), joint="J4", note="bolts to the forearm hub ring's inner flange")
    b = P.BEARINGS["wrist_flex"][0]
    bearing_y("j4_bearing", "wrist", b, y_w + 1.0, 0, 0, "J4")
    hub = ring_y(b[1] / 2, b[1] / 2 - 3.0, y_w - 2.0, y_w + 1.0 + b[3]).union(ring_y(b[1] / 2, 10.5, y_w - 2.0, y_w))
    add("j4_forearm_hub", "wrist", "metal", hub, bom=("turned aluminium hub ring (on the forearm)", 1), joint="J4",
        note="part of the forearm: inner race of the 6806, the J4 horn bolts to its flange; the bearing carries the wrist moment")
    # J5: STS3032 micro servo off-axis, 1:1 m0.5 gears to the roll shaft on the axis
    g = P.GEARS["wrist_roll"]
    m, zg = g["module"], g["z_hub"]
    cd = m * zg  # equal gears -> centre distance = pitch diameter
    Lm, Wm, Hm = P.STS3032.body
    y5 = yc + cd
    z_gear0, z_gear1 = 38.0, 38.0 + g["face"]
    mc = box(-Lm / 2, Lm / 2, y5 - Wm / 2, y5 + Wm / 2, z_gear0 - 4.5 - Hm, z_gear0 - 4.5)
    add("j5_servo", "wrist", "servo", mc, bom=("Feetech STS3032 6 V micro bus servo", 1), joint="J5",
        note="size VERIFY: vendors list 23.2x12.1x28.5 and 32x12x27.5")
    add("j5_servo_spline", "wrist", "metal", cyl(2.5, (0, y5, z_gear0 - 4.5), (0, y5, z_gear0)), joint="J5")
    add("j5_pinion", "wrist", "steel_gear", gear_z(zg, m, z_gear0, z_gear1, 0, y5, bore=1.5, phase=mesh_phase(math.pi / 2, zg, zg)), bom=(f"brass/steel gear m{m:g} {zg}T", 1), joint="J5")
    add("j5_roll_gear", "wrist", "steel_gear", gear_z(zg, m, z_gear0, z_gear1, 0, yc, bore=3.0), bom=(f"brass/steel gear m{m:g} {zg}T", 1), joint="J5")
    rb = P.BEARINGS["wrist_roll"][0]
    shaft = cyl(6.0, (0, yc, z_gear0 - 2), (0, yc, A["wrist"] - 4.0)).union(cyl(rb[1] / 2, (0, yc, A["wrist"] - 11.0), (0, yc, A["wrist"] - 4.0)))
    add("j5_roll_shaft", "wrist", "metal", shaft, bom=("turned aluminium roll shaft", 1), joint="J5", note="bolts to the j5 roll flange; the bearing carries the tool moment")
    bearing_z("j5_bearing", "wrist", rb, A["wrist"] - 11.0, 0, yc, "J5")
    add("j5_buck_6v", "wrist", "pcb", box(-24.0, -20.0, yc - 6.0, yc + 8.0, 4.0, 22.0), bom=("mini buck 12 V -> 6 V, 2 A", 1), joint="J5",
        note="feeds only the STS3032; data line shared with the 12 V bus (TTL levels)")


# ----------------------------------------------------------------------------- cutaway shells


Y_CUT = 31.0  # just past the gear layer (gears y 20..30): the face-on view of both gear trains


def halves(shape, axis):
    """(kept, removed) halves: axis 'x' keeps x <= 0, axis 'y' keeps y <= Y_CUT."""
    big = 500
    if axis == "x":
        keep, rem = box(-big, 0, -big, big, -big, big), box(0, big, -big, big, -big, big)
    else:
        keep, rem = box(-big, big, -big, Y_CUT, -big, big), box(-big, big, Y_CUT, big, -big, big)
    out = []
    for h in (keep, rem):
        try:
            sh = shape.intersect(h)
            out.append(sh if sh.val().Volume() > 1 else None)
        except Exception:
            out.append(None)
    return out


def arm_shells():
    import parts as PM

    PM.build_all()
    return [(p.name, p.body, p.role, p.shape) for p in PM.PARTS
            if (p.group == "arm" or p.name.startswith(("j5_roll_flange", "coupler_robot"))) and not p.name.startswith(("tool_", "coupler_tool", "coupler_contact"))]


def export_cuts(items, axis, tag):
    d = HERE / "out" / f"cut_{axis}"
    for sub in ("keep", "ghost"):
        (d / sub).mkdir(parents=True, exist_ok=True)
    man = []
    for name, body, role, shape in items:
        k, r = halves(shape, axis)
        for sub, sh in (("keep", k), ("ghost", r)):
            if sh is not None:
                cq.exporters.export(sh, str(d / sub / f"{name}.stl"), tolerance=0.05, angularTolerance=0.12)
                man.append(dict(name=name, body=body, role=role, part=sub, src=tag))
    return man


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    build_j1()
    build_j2_j3()
    build_wrist()
    man = []
    for p in PARTS:
        cq.exporters.export(p["shape"], str(OUT / f"{p['name']}.stl"), tolerance=0.03, angularTolerance=0.1)
        v = p["shape"].val()
        man.append(dict(name=p["name"], body=p["body"], role=p["role"], joint=p["joint"], note=p["note"], bom=p["bom"], valid=v.isValid()))
    (OUT / "manifest.json").write_text(json.dumps(man, indent=1))
    shells = arm_shells()
    cx = export_cuts(shells, "x", "shell")  # internals stay whole in the x-section views
    cy = export_cuts(shells, "y", "shell") + export_cuts([(p["name"], p["body"], p["role"], p["shape"]) for p in PARTS], "y", "internal")
    (HERE / "out" / "cut_x" / "manifest.json").write_text(json.dumps(cx, indent=1))
    (HERE / "out" / "cut_y" / "manifest.json").write_text(json.dumps(cy, indent=1))
    print(len(man), "internal parts; invalid:", [m["name"] for m in man if not m["valid"]], f"; cut_x {len(cx)} / cut_y {len(cy)} pieces")


if __name__ == "__main__":
    main()
