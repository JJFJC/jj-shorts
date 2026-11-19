"""Gantry Arm CAD, industrial-cobot design language (original design).

Every part is built in the frame of the MuJoCo body it rides on (see sim/model.py), so
the same forward kinematics place the CAD for renders and the assembly STEP.

Design rules:
    - pitch joints are round "pucks" whose axis is the joint axis (local Y)
    - links are hollow capsule side-plates offset beside the pucks (UR-style layout)
    - servos sit fully inside pucks / housings; wiring runs through hollow links and
      14 mm bores at every joint centre
    - shells split on a fillet tangent line (seam reads as a design line); M3 screws
      enter from the inner face only inside the shadow of the neighbouring puck (hub zones)
      into heat-set inserts; the visible mid-span is held by snap clips
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cadquery as cq
from cadquery import Vector as V

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import params as P  # noqa: E402

S = P.STYLE
A = P.ARM
WALL = S["wall"]


# ============================================================================ helpers


def W(solid) -> cq.Workplane:
    return cq.Workplane("XY").add(solid)


def cyl(r, p0, p1) -> cq.Workplane:
    d = V(*p1) - V(*p0)
    return W(cq.Solid.makeCylinder(r, d.Length, V(*p0), d.normalized()))


def cone(r0, r1, p0, p1) -> cq.Workplane:
    d = V(*p1) - V(*p0)
    return W(cq.Solid.makeCone(r0, r1, d.Length, V(*p0), d.normalized()))


def box(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    return W(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0)))


def fil(wp: cq.Workplane, r: float, sel: str | None = None) -> cq.Workplane:
    """Fillet that falls back to the unfilleted shape instead of crashing the build."""
    try:
        e = wp.edges(sel) if sel else wp.edges()
        return e.fillet(r)
    except Exception:
        try:
            e = wp.edges(sel) if sel else wp.edges()
            return e.fillet(r * 0.5)
        except Exception:
            return wp


def rbox(x0, x1, y0, y1, z0, z1, r) -> cq.Workplane:
    return fil(box(x0, x1, y0, y1, z0, z1), r)


def puck(r, y0, y1, edge_r) -> cq.Workplane:
    return fil(cyl(r, (0, y0, 0), (0, y1, 0)), edge_r, "%CIRCLE")


def stadium(r1, r2, L, y0, y1, edge_r=0.0) -> cq.Workplane:
    """Tapered capsule plate: hull of circle r1 at z=0 and r2 at z=L (XZ plane), from y0 to y1."""
    sa = (r1 - r2) / L
    ca = math.sqrt(1 - sa * sa)
    T = y1 - y0
    wp = (
        cq.Workplane("XY")
        .moveTo(r1 * ca, r1 * sa)
        .lineTo(r2 * ca, L + r2 * sa)
        .threePointArc((0, L + r2), (-r2 * ca, L + r2 * sa))
        .lineTo(-r1 * ca, r1 * sa)
        .threePointArc((0, -r1), (r1 * ca, r1 * sa))
        .close()
        .extrude(T)
    )
    if edge_r:
        wp = fil(wp, edge_r, "not |Z")
    # (x, y2d, t) -> (x, -t, y2d), then shift so thickness spans y0..y1
    return wp.rotate((0, 0, 0), (1, 0, 0), 90).translate((0, y1, 0))


def bar(p0, p1, w, y0, y1) -> cq.Workplane:
    """Flat link with round ends in the XZ plane between p0=(x,z) and p1=(x,z), thickness y0..y1."""
    (x0, z0), (x1, z1) = p0, p1
    L = math.hypot(x1 - x0, z1 - z0)
    ang = math.degrees(math.atan2(x1 - x0, z1 - z0))
    b = stadium(w / 2, w / 2, L, y0, y1, 0.0)
    return b.rotate((0, 0, 0), (0, 1, 0), ang).translate((x0, 0, z0))


# ============================================================================ registry


@dataclass
class Part:
    name: str
    body: str  # MuJoCo body frame the part lives in ("world" = fixed)
    role: str  # material role for renders: shell cap accent extrusion strip metal rubber smoke lens brass estop_red estop_yellow belt
    shape: cq.Workplane
    qty: int = 1
    printed: bool = True
    material: str = "PETG"
    orient: str = ""  # print orientation note
    fill: float = 1.0  # printed mass factor (1.0 = thin-wall shell printed solid)
    group: str = "arm"
    hidden_in_render: bool = False
    extra: dict = field(default_factory=dict)


PARTS: list[Part] = []


def reg(*a, **k) -> Part:
    p = Part(*a, **k)
    PARTS.append(p)
    return p


# ============================================================================ split shell helper


def split_shell(outer, inner, y_split, keep_cover_side: str, bosses: list, y_inner_wall: float, y_outer_wall: float):
    """Hollow `outer` by `inner`, split at y=y_split, add screw bosses (x, z) joining the two halves.

    keep_cover_side: '+' if the cover is the y > y_split side.
    Returns (body, cover).
    """
    shell = outer.cut(inner)
    big = 1000
    if keep_cover_side == "+":
        body = shell.intersect(box(-big, big, -big, y_split, -big, big))
        cover = shell.intersect(box(-big, big, y_split, big, -big, big))
        yb0, yb1, yc0, yc1 = y_inner_wall, y_split, y_split, y_outer_wall
    else:
        body = shell.intersect(box(-big, big, y_split, big, -big, big))
        cover = shell.intersect(box(-big, big, -big, y_split, -big, big))
        yb0, yb1, yc0, yc1 = y_split, y_inner_wall, y_outer_wall, y_split
    for x, z in bosses:
        cover = cover.union(cyl(4.5, (x, min(yc0, yc1), z), (x, max(yc0, yc1), z)))
        body = body.union(cyl(4.5, (x, min(yb0, yb1), z), (x, max(yb0, yb1), z)))
    # clearance holes for M3 through the inner wall (screw heads hidden against the pucks)
    for x, z in bosses:
        if keep_cover_side == "+":
            body = body.cut(cyl(1.7, (x, y_inner_wall - WALL - 1, z), (x, y_split + 0.1, z)))
            body = body.cut(cyl(3.0, (x, y_inner_wall - WALL - 1, z), (x, y_inner_wall - WALL + 1.8, z)))
        else:
            body = body.cut(cyl(1.7, (x, y_split - 0.1, z), (x, y_inner_wall + WALL + 1, z)))
            body = body.cut(cyl(3.0, (x, y_inner_wall + WALL - 1.8, z), (x, y_inner_wall + WALL + 1, z)))
    # cover boss holes for heat-set inserts (M3 x 4 insert, 4 mm hole)
    for x, z in bosses:
        if keep_cover_side == "+":
            cover = cover.cut(cyl(2.0, (x, y_split - 0.1, z), (x, y_outer_wall - 1.0, z)))
        else:
            cover = cover.cut(cyl(2.0, (x, y_outer_wall + 1.0, z), (x, y_split + 0.1, z)))
    return body, cover


# ============================================================================ arm


def hub_bosses(L: float, r_a: float, r_b: float) -> list:
    """Screw positions hidden behind the neighbouring pucks: two per hub, inside the puck shadow
    (radius r_a at z=0, r_b at z=L) and clear of the 14 mm wiring bore."""
    da, db = 0.5 * r_a, 0.5 * r_b
    return [(-da * 0.8, da * 0.6), (da * 0.8, da * 0.6), (-db * 0.8, L - db * 0.6), (db * 0.8, L - db * 0.6)]


def add_snap_clips(body, cover, L, r_mid, y_split, cover_side: str):
    """Two mid-span snap clips: a flexible tongue on the cover hooks under a ridge in the body.
    No screws in the visible middle of the link."""
    sgn = 1 if cover_side == "+" else -1
    for sx in (-1, 1):
        x_in = sx * (r_mid - WALL - 1.2)  # just inside the side wall
        x0, x1 = sorted((x_in, x_in - sx * 1.6))
        y0, y1 = sorted((y_split + sgn * 1.0, y_split - sgn * 6.0))
        tongue = box(x0, x1, y0, y1, L / 2 - 5, L / 2 + 5)
        hook = box(*sorted((x_in - sx * 1.6, x_in - sx * 2.6)), *sorted((y_split - sgn * 4.6, y_split - sgn * 6.0)), L / 2 - 5, L / 2 + 5)
        cover = cover.union(tongue).union(hook)
        ridge = box(*sorted((x_in - sx * 1.7, x_in + sx * 0.4)), *sorted((y_split - sgn * 6.2, y_split - sgn * 7.4)), L / 2 - 7, L / 2 + 7)
        body = body.union(ridge)
    return body, cover




def elbow_width() -> float:
    return 82.0 if P.ELBOW_SERVOS == 2 else 48.0


def wrist_center_y() -> float:
    return 48.0 - elbow_width() + 32.0


def build_arm():
    T = S["link_thick"]
    g = S["gap"]
    fr = S["fillet"]
    rs, ws = S["shoulder_puck"]

    # ---- J1 fixed base on the platform (z_carriage frame, J1 axis at y = platform offset)
    off = P.PLATFORM["offset"]
    base = fil(cyl(60, (0, off, 0), (0, off, 34)), 8, ">Z")
    base = base.cut(cyl(7, (0, off, -1), (0, off, 35)))  # wiring bore through the J1 centre
    base = base.cut(cyl(60 - WALL, (0, off, -1), (0, off, 34 - WALL)).cut(cyl(16, (0, off, -2), (0, off, 40))))  # hollow, keep a centre boss
    reg("j1_base_housing", "z_carriage", "shell", base, orient="top face down, no supports", fill=1.0,
        extra=dict(note="holds STS3215 (J1) + 6 mm-thick lazy-susan / thrust bearing ring"))

    # ---- turret: collar + column + shoulder puck (rotates with J1)
    collar = fil(cyl(56, (0, 0, 34.5), (0, 0, 42)), 1.5, "%CIRCLE").cut(cyl(7, (0, 0, 34), (0, 0, 43)))
    reg("j1_turret_collar", "turret", "cap", collar, orient="flat, no supports", fill=0.6)
    zj2 = A["base_h"]
    col = cyl(46, (0, 0, 42), (0, 0, zj2))
    pk = puck(rs, -ws / 2, ws / 2, 6).translate((0, 0, zj2))
    turret = col.union(pk)
    inner = cyl(46 - WALL, (0, 0, 42 - 1), (0, 0, zj2)).union(cyl(rs - WALL, (0, -ws / 2 + WALL, zj2), (0, ws / 2 - WALL, zj2)))
    turret = turret.cut(inner)
    # recess for the two end caps
    for sgn in (1, -1):
        turret = turret.cut(cyl(rs - 7.8, (0, sgn * (ws / 2 - 2.0), zj2), (0, sgn * (ws / 2 + 1), zj2)))
    reg("j2_shoulder_puck", "turret", "shell", turret, orient="axis vertical (Y up), tree supports inside puck only", fill=1.0,
        extra=dict(note="houses 2x STS3250 (J2) facing outward; split vertically at the column/puck seam if printing is hard"))
    for sgn, nm in ((1, "j2_cap_outer"), (-1, "j2_cap_inner")):
        c = fil(cyl(rs - 8.0, (0, sgn * (ws / 2 - 2.0), zj2), (0, sgn * ws / 2, zj2)), 0.8, "%CIRCLE")
        c = c.cut(cyl(7, (0, sgn * (ws / 2 - 3), zj2), (0, sgn * (ws / 2 + 1), zj2)))
        reg(nm, "turret", "cap", c, orient="flat, face down on textured plate", fill=1.0)

    # ---- upper arm: offset capsule side-link on +Y, beside the shoulder and elbow pucks
    r1, r2 = S["link_r"]["upper"]
    L = A["upper"]
    y0 = ws / 2 + g
    y1 = y0 + T
    outer = stadium(r1, r2, L, y0, y1, fr)
    inner = stadium(r1 - WALL, r2 - WALL, L, y0 + WALL, y1 - WALL, 0)
    led = S["led"]
    body, cover = split_shell(outer, inner, y1 - fr, "+", hub_bosses(L, rs, S["elbow_puck_r"]), y0 + WALL, y1 - WALL)
    body, cover = add_snap_clips(body, cover, L, (r1 + r2) / 2, y1 - fr, "+")
    for z in (0, L):  # wiring bores at joint centres on the inner face
        body = body.cut(cyl(7, (0, y0 - 1, z), (0, y0 + WALL + 1, z)))
    cover = cover.cut(box(-led["width"] / 2, led["width"] / 2, y1 - led["depth"], y1 + 1, led["z0"], led["z1"]))
    reg("upper_arm_body", "upper_arm", "shell", body, orient="inner face down (flat), supports in hub bores only", fill=1.0)
    reg("upper_arm_cover", "upper_arm", "shell", cover, orient="outer face down on smooth PEI (best surface)", fill=1.0,
        extra=dict(note="LED channel for WS2812 strip (60/m, 6 LEDs) under a diffuser"))
    diff = fil(box(-led["width"] / 2 + 0.2, led["width"] / 2 - 0.2, y1 - led["depth"] + 0.1, y1 - 0.15, led["z0"] + 0.5, led["z1"] - 0.5), 0.5, "|Y")
    reg("upper_arm_led_diffuser", "upper_arm", "accent", diff, material="PETG natural (translucent)", orient="on edge", fill=1.0)

    # ---- forearm: elbow puck + offset capsule side-link on -Y
    we = elbow_width()
    re_ = S["elbow_puck_r"]
    ey0, ey1 = 48.0 - we, 48.0
    ep = puck(re_, ey0, ey1, 5).cut(cyl(re_ - WALL, (0, ey0 + WALL, 0), (0, ey1 - WALL, 0)))
    for yy, sgn in ((ey1, 1), (ey0, -1)):
        ep = ep.cut(cyl(re_ - 6.8, (0, yy - sgn * 2.0, 0), (0, yy + sgn * 1, 0)))
    reg("j3_elbow_puck", "forearm", "shell", ep, orient="axis vertical, supports inside only", fill=1.0,
        extra=dict(note=f"houses {P.ELBOW_SERVOS}x STS3250 (J3)"))
    for yy, sgn, nm in ((ey1, 1, "j3_cap_a"), (ey0, -1, "j3_cap_b")):
        c = fil(cyl(re_ - 7.0, (0, yy - sgn * 2.0, 0), (0, yy, 0)), 0.8, "%CIRCLE").cut(cyl(7, (0, yy - sgn * 3, 0), (0, yy + sgn, 0)))
        reg(nm, "forearm", "cap", c, orient="flat, face down", fill=1.0)
    r1, r2 = S["link_r"]["fore"]
    L = A["fore"]
    fy1 = ey0 - g
    fy0 = fy1 - T
    outer = stadium(r1, r2, L, fy0, fy1, fr)
    inner = stadium(r1 - WALL, r2 - WALL, L, fy0 + WALL, fy1 - WALL, 0)
    body, cover = split_shell(outer, inner, fy0 + fr, "-", hub_bosses(L, re_, S["wrist_puck"][0]), fy1 - WALL, fy0 + WALL)
    body, cover = add_snap_clips(body, cover, L, (r1 + r2) / 2, fy0 + fr, "-")
    for z in (0, L):
        body = body.cut(cyl(7, (0, fy1 - WALL - 1, z), (0, fy1 + 1, z)))
    reg("forearm_body", "forearm", "shell", body, orient="inner face down, supports in hub bores only", fill=1.0)
    reg("forearm_cover", "forearm", "shell", cover, orient="outer face down on smooth PEI", fill=1.0)

    # ---- wrist: J4 pitch puck + J5 roll housing (one shell, split into front/back halves)
    rw, ww = S["wrist_puck"]
    yc = wrist_center_y()
    rr = S["roll_r"]
    wp = puck(rw, yc - ww / 2, yc + ww / 2, 4)
    roll = fil(cyl(rr, (0, yc, 0), (0, yc, A["wrist"] - 4)), 3, ">Z")
    wr = wp.union(roll)
    wr_in = cyl(rw - WALL, (0, yc - ww / 2 + WALL, 0), (0, yc + ww / 2 - WALL, 0)).union(cyl(rr - WALL, (0, yc, 0), (0, yc, A["wrist"] - 4 - WALL)))
    wr = wr.cut(wr_in).cut(cyl(7, (0, yc, A["wrist"] - 10), (0, yc, A["wrist"])))
    for sgn in (1, -1):
        wr = wr.cut(cyl(rw - 6.8, (0, yc + sgn * (ww / 2 - 2.0), 0), (0, yc + sgn * (ww / 2 + 1), 0)))
    big = 500
    front = wr.intersect(box(0, big, -big, big, -big, big))
    back = wr.intersect(box(-big, 0, -big, big, -big, big))
    for nm, half in (("wrist_shell_front", front), ("wrist_shell_back", back)):
        reg(nm, "wrist", "shell", half, orient="split face down (flat), no supports", fill=1.0,
            extra=dict(note="halves join on the XZ mid-plane with 4x M3; houses STS3215 J4 + STS3215 J5"))
    for sgn, nm in ((1, "j4_cap_a"), (-1, "j4_cap_b")):
        c = fil(cyl(rw - 7.0, (0, yc + sgn * (ww / 2 - 2.0), 0), (0, yc + sgn * ww / 2, 0)), 0.8, "%CIRCLE")
        reg(nm, "wrist", "cap", c, orient="flat, face down", fill=1.0)
    return yc


# ============================================================================ coupler + tools


def coupler_robot(yc):
    rr = S["roll_r"]
    flange = fil(cyl(rr, (0, yc, -4), (0, yc, 0)), 0.8, "%CIRCLE")
    reg("j5_roll_flange", "tool", "cap", flange, orient="flat", fill=0.8)
    h = fil(cyl(rr, (0, yc, 0), (0, yc, 8)), 0.8, "<Z").cut(cone(8.5, 6.0, (0, yc, 8.01), (0, yc, 3)))
    mag = []
    for ang in (90, 210, 330):
        x, y = 19 * math.cos(math.radians(ang)), yc + 19 * math.sin(math.radians(ang))
        h = h.cut(cyl(4.1, (x, y, 5), (x, y, 8.1)))
        mag.append(cyl(4.0, (x, y, 5), (x, y, 7.8)))
    pins = []
    for dx in (-4.5, -1.5, 1.5, 4.5):
        pins.append(cyl(0.9, (dx, yc - 13, 4), (dx, yc - 13, 9.0)))
        h = h.cut(cyl(1.25, (dx, yc - 13, 2), (dx, yc - 13, 8.1)))
    h = h.cut(cyl(5, (0, yc, -1), (0, yc, 3.5)))  # wiring bore into the roll housing
    reg("coupler_robot_half", "tool", "shell", h, orient="flat, magnet face up", fill=0.7,
        extra=dict(note="3x 8x3 mm N52 magnets, 4x pogo pins (12V, GND, DATA, TOOL_ID), cone socket"))
    m = mag[0]
    for x in mag[1:]:
        m = m.union(x)
    reg("coupler_magnets", "tool", "metal", m, printed=False, material="N52 8x3 mm", qty=1)
    pn = pins[0]
    for x in pins[1:]:
        pn = pn.union(x)
    reg("coupler_pogo_pins", "tool", "brass", pn, printed=False, material="pogo pin 2 mm, 4 per robot half")
    # locking sleeve: quarter-turn bayonet collar with a lever tab
    sl = cyl(rr + 2.6, (0, yc, 1), (0, yc, 11)).cut(cyl(rr + 0.25, (0, yc, 0), (0, yc, 12)))
    tab = rbox(rr, rr + 11, yc - 5, yc + 5, 3, 9, 2.4)
    sl = fil(sl, 0.8, "%CIRCLE").union(tab)
    reg("coupler_lock_sleeve", "tool", "cap", sl, orient="flat, no supports", fill=0.9,
        extra=dict(note="quarter-turn bayonet: 3 internal ramps engage lugs on the tool half (mechanical lock)"))
    # wrist camera pod on the robot half (rotates with J5, looks along the tool axis)
    pod = rbox(rr + 0.5, rr + 13, yc - 10, yc + 10, -10, 8, 3.5)
    lens = cyl(4.2, (rr + 7, yc, 8), (rr + 7, yc, 9.6))
    reg("wrist_camera_pod", "tool", "shell", pod, orient="back face down", fill=0.6, extra=dict(note="fits a 32x32 mm USB UVC board"))
    reg("wrist_camera_lens", "tool", "lens", lens, printed=False, material="camera lens")


def coupler_tool_half(yc, z0=8.0):
    rr = S["roll_r"]
    h = fil(cyl(rr, (0, yc, z0), (0, yc, z0 + 8)), 1.2, ">Z")
    h = h.union(cone(8.2, 5.7, (0, yc, z0), (0, yc, z0 - 4.6)))
    for ang in (30, 150, 270):  # bayonet lugs under the sleeve
        a = math.radians(ang)
        h = h.union(box(-3, 3, -2, 2, z0 + 0.5, z0 + 4).rotate((0, 0, 0), (0, 0, 1), ang).translate((rr * math.cos(a), yc + rr * math.sin(a), 0)))
    for dx in (-4.5, -1.5, 1.5, 4.5):
        h = h.cut(cyl(1.3, (dx, yc - 13, z0 - 0.1), (dx, yc - 13, z0 + 0.6)))
    return h


def build_tools(yc):
    rr = S["roll_r"]
    reg("coupler_tool_half", "tool", "shell", coupler_tool_half(yc), qty=4, orient="flat, cone up, no supports", fill=0.7,
        extra=dict(note="one per tool; 3 steel discs face the magnets, 4 brass contact pads"))
    reg("coupler_contact_pads", "tool", "brass",
        cyl(1.2, (-4.5, yc - 13, 8.0), (-4.5, yc - 13, 8.5)).union(cyl(1.2, (-1.5, yc - 13, 8.0), (-1.5, yc - 13, 8.5)))
        .union(cyl(1.2, (1.5, yc - 13, 8.0), (1.5, yc - 13, 8.5))).union(cyl(1.2, (4.5, yc - 13, 8.0), (4.5, yc - 13, 8.5))),
        printed=False, material="brass pads / PCB, 4 per tool", qty=4)

    # ---- parallel gripper: cylindrical housing, smoked cap, V-linkage steel fingers, rubber pads
    z0 = 16.0
    house = fil(cyl(rr + 1, (0, yc, z0), (0, yc, z0 + 38)), 3, "%CIRCLE")
    house = house.cut(cyl(rr + 1 - WALL, (0, yc, z0 + WALL), (0, yc, z0 + 39)))
    reg("gripper_housing", "tool", "shell", house, orient="open end up, no supports", fill=1.0,
        extra=dict(note="STS3215 lies across the housing; pinion drives a twin rack"))
    cap = fil(cyl(rr - 1, (0, yc, z0 + 38), (0, yc, z0 + 46)), 3.5, ">Z")
    cap = cap.cut(cyl(rr - 1 - 1.6, (0, yc, z0 + 37), (0, yc, z0 + 44.4)))
    for sgn in (1, -1):
        cap = cap.cut(box(min(sgn * 4, sgn * 26), max(sgn * 4, sgn * 26), yc - 9.5, yc + 9.5, z0 + 40, z0 + 47))
    reg("gripper_smoke_cap", "tool", "smoke", cap, material="PETG translucent grey (or smoked PETG)", orient="dome down, 100% infill, slow outer wall", fill=1.0)
    mech = cyl(8, (0, yc - 4, z0 + 40), (0, yc + 4, z0 + 40)).union(box(-14, 14, yc - 3, yc + 3, z0 + 33, z0 + 36))
    reg("gripper_pinion_rack", "tool", "metal", mech, printed=False, material="M0.5 pinion + rack (or printed PETG)")
    fing = None
    pins = None
    pads = None
    for sgn in (1, -1):
        zp, zt = z0 + 42, z0 + 58  # pivot height, tip joint height
        for yy0, yy1 in ((yc - 9, yc - 6), (yc + 6, yc + 9)):
            for xa in (6.0, 21.0):
                b = bar((sgn * xa, zp), (sgn * 18.5, zt), 6.0, yy0, yy1)
                fing = b if fing is None else fing.union(b)
        for xa, zz in ((6.0, zp), (21.0, zp), (18.5, zt)):
            pn = cyl(1.6, (sgn * xa, yc - 10.5, zz), (sgn * xa, yc + 10.5, zz))
            pins = pn if pins is None else pins.union(pn)
        plate = rbox(min(sgn * 16.5, sgn * 21), max(sgn * 16.5, sgn * 21), yc - 9, yc + 9, z0 + 55, z0 + 78, 1.2)
        fing = fing.union(plate)
        pad = rbox(min(sgn * 13.8, sgn * 16.6), max(sgn * 13.8, sgn * 16.6), yc - 8, yc + 8, z0 + 58, z0 + 76, 1.0)
        pads = pad if pads is None else pads.union(pad)
    reg("gripper_fingers_vlink", "tool", "metal", fing, printed=False, material="2 mm stainless laser-cut links (or PETG + metallic paint)", qty=1,
        extra=dict(note="2 fingers x (2 V-links pairs + finger plate)"))
    reg("gripper_pivot_pins", "tool", "metal", pins, printed=False, material="3 mm steel dowel pins")
    reg("gripper_pads", "tool", "rubber", pads, printed=False, material="TPU 95A pads (or 2 mm silicone sheet)")

    # ---- other tools (shown in the tool lineup render, not mounted)
    sx = 0.0
    suc = fil(cyl(24, (sx, yc, z0), (sx, yc, z0 + 30)), 3, "%CIRCLE").union(cyl(4, (sx, yc, z0 + 30), (sx, yc, z0 + 58)))
    cup = cone(5, 13, (sx, yc, z0 + 58), (sx, yc, z0 + 70))
    reg("tool_suction_body", "tool_suction", "shell", suc, orient="flat end down", fill=0.6,
        extra=dict(note="holds a 370-size 12 V mini vacuum pump + solenoid; power via pogo 12 V"))
    reg("tool_suction_cup", "tool_suction", "rubber", cup, printed=False, material="20 mm bellows suction cup")
    pen = fil(cyl(14, (0, yc, z0), (0, yc, z0 + 52)), 2, "%CIRCLE").cut(cyl(6, (0, yc, z0 + 10), (0, yc, z0 + 53)))
    pen = pen.union(fil(box(12, 22, yc - 5, yc + 5, z0 + 30, z0 + 42), 2))
    reg("tool_pen_holder", "tool_pen", "shell", pen, orient="flat end down", fill=0.6,
        extra=dict(note="spring-loaded sleeve, side thumb screw; fits 8-12 mm pens"))
    reg("tool_pen_marker", "tool_pen", "accent", cyl(5.5, (0, yc, z0 + 20), (0, yc, z0 + 78)).union(cone(5.5, 1.2, (0, yc, z0 + 78), (0, yc, z0 + 88))), printed=False, material="marker (not supplied)")
    ph = fil(box(-12, 12, yc - 12, yc + 12, z0, z0 + 30), 3).union(fil(box(-48, 48, yc - 8, yc + 8, z0 + 30, z0 + 40), 3))
    for sgn in (1, -1):
        ph = ph.union(fil(box(min(sgn * 40, sgn * 48), max(sgn * 40, sgn * 48), yc - 8, yc + 8, z0 + 40, z0 + 60), 2))
    reg("tool_phone_mount", "tool_phone", "shell", ph, orient="clamp face down", fill=0.6,
        extra=dict(note="spring jaw clamps phones 60-90 mm wide"))
    reg("tool_phone_pads", "tool_phone", "rubber", box(-40.5, -38.5, yc - 7, yc + 7, z0 + 44, z0 + 58).union(box(38.5, 40.5, yc - 7, yc + 7, z0 + 44, z0 + 58)), printed=False, material="TPU pads")


# ============================================================================ gantry


def build_gantry():
    L = P.X_RAIL_LEN
    # ---- X rail 2060 lying flat on the table (world frame)
    rail = box(-L / 2, L / 2, -30, 30, 0, 20)
    for y in (-20, 0, 20):
        rail = rail.cut(box(-L / 2 - 1, L / 2 + 1, y - 3.1, y + 3.1, 14, 21))
    for sgn in (1, -1):
        rail = rail.cut(box(-L / 2 - 1, L / 2 + 1, min(sgn * 24, sgn * 31), max(sgn * 24, sgn * 31), 6.9, 13.1))
    reg("x_rail_2060", "world", "extrusion", rail, printed=False, material="2060 V-slot 1000 mm, black anodised", group="gantry", fill=0.39)
    strips = box(-L / 2, L / 2, -23.1, -16.9, 18.6, 19.8).union(box(-L / 2, L / 2, 16.9, 23.1, 18.6, 19.8))
    reg("x_rail_cover_strips", "world", "strip", strips, printed=False, material="V-slot cover strip (PVC), 2x 1 m", group="gantry")
    reg("x_belt", "world", "belt", box(-L / 2 + 30, L / 2 - 30, -3, 3, 15, 16.6), printed=False, material="GT2 6 mm belt", group="gantry")

    # end housings: X motor (-X) and idler (+X)
    mot = rbox(-L / 2 - 78, -L / 2 + 2, -74, 52, 0, 74, 10).cut(box(-L / 2 - 75.6, -L / 2 - 0.4, -71.6, 49.6, -1, 71.6))
    reg("x_motor_housing", "world", "shell", mot, orient="open bottom down", fill=1.0, group="gantry",
        extra=dict(note="NEMA17 vertical, GT2 20T pulley; printed as 2 halves split at x=-540 (lap joint) if needed"))
    idl = rbox(L / 2 - 2, L / 2 + 38, -42, 42, 0, 44, 8).cut(box(L / 2 - 3, L / 2 + 35.6, -39.6, 39.6, -1, 41.6))
    reg("x_idler_cap", "world", "cap", idl, orient="open bottom down", fill=0.9, group="gantry")
    # cable trough behind the rail (hides the X cable chain)
    seg = 240.0
    for i in range(4):
        x0 = -L / 2 + 20 + i * seg
        t = rbox(x0 + 0.3, x0 + seg - 0.3, -160, -82, 0, 64, 6)
        t = t.cut(box(x0 - 1, x0 + seg + 1, -157.6, -84.4, 2.4, 61.6))  # 2.4 mm walls, open-ended segment
        t = t.cut(box(x0 - 1, x0 + seg + 1, -124, -116, 59, 65))  # brush slot for the chain arm
        reg(f"x_cable_trough_{i + 1}", "world", "shell", t, orient="upside down (top on bed)", fill=1.0, group="gantry",
            extra=dict(note="4 segments, hidden lap joint, brush strip in top slot"))
    # controller enclosure with hardware e-stop
    ctrl = rbox(-L / 2 - 170, -L / 2 - 84, -160, -40, 0, 82, 10).cut(box(-L / 2 - 167.6, -L / 2 - 86.4, -157.6, -42.4, -1, 79.6))
    reg("controller_enclosure", "world", "shell", ctrl, orient="open bottom down", fill=1.0, group="gantry",
        extra=dict(note="ESP32 + 2x TMC2209 + Waveshare bus adapter + 12/24 V distribution"))
    cx, cy = -L / 2 - 127, -100
    reg("estop_base", "world", "estop_yellow", fil(cyl(22, (cx, cy, 82), (cx, cy, 92)), 2, ">Z"), printed=False, material="22 mm e-stop", group="gantry")
    reg("estop_button", "world", "estop_red", fil(cyl(17, (cx, cy, 92), (cx, cy, 106)), 6, ">Z"), printed=False, material="22 mm e-stop", group="gantry")

    # ---- X carriage shroud + column (x_carriage frame: origin on rail top at column centre)
    sh = rbox(-78, 78, -74, 60, -19, 62, 12)
    sh = sh.cut(box(-80, 80, -31, 31, -20, 0.5))  # rail channel
    sh = sh.cut(box(-75, 75, -71, 57, -20, 59))  # hollow
    reg("x_carriage_shroud", "x_carriage", "shell", sh, orient="top face down", fill=1.0, group="gantry",
        extra=dict(note="covers 6 V-wheels, Z motor (NEMA17 under the column) and the column foot"))
    arm_fin = rbox(-12, 12, -122, -72, 48, 60, 3)
    reg("x_chain_arm", "x_carriage", "shell", arm_fin, orient="flat", fill=0.8, group="gantry")
    H = P.Z_COLUMN_LEN
    col = box(-10, 10, -20, 20, 10, 10 + H)
    for y in (-10, 10):
        for sgn in (1, -1):
            col = col.cut(box(min(sgn * 6.5, sgn * 11), max(sgn * 6.5, sgn * 11), y - 3.1, y + 3.1, 9, 11 + H))
    for sgn in (1, -1):
        col = col.cut(box(-3.1, 3.1, min(sgn * 16.5, sgn * 21), max(sgn * 16.5, sgn * 21), 9, 11 + H))
    reg("z_column_2040", "x_carriage", "extrusion", col, printed=False, material="2040 V-slot 700 mm, black anodised", group="gantry", fill=0.40)
    zs = None
    for y in (-10, 10):
        for sgn in (1, -1):
            s_ = box(min(sgn * 9.3, sgn * 10.0), max(sgn * 9.3, sgn * 10.0), y - 3.0, y + 3.0, 62, 10 + H)
            zs = s_ if zs is None else zs.union(s_)
    reg("z_column_cover_strips", "x_carriage", "strip", zs, printed=False, material="V-slot cover strip (PVC), 4x 650 mm", group="gantry")
    cap = rbox(-26, 26, -64, 24, 10 + H, 10 + H + 22, 7)
    reg("z_column_top_cap", "x_carriage", "cap", cap, orient="top down", fill=0.8, group="gantry", extra=dict(note="holds the T8x2 top bearing (608ZZ)"))
    for i, (z0, z1) in enumerate(((62, 278), (278, 494), (494, 10 + H))):
        rc = rbox(-24, 24, -64, -21, z0 + 0.3, z1 - 0.3, 5).cut(box(-21.6, 21.6, -61.6, -20, z0 - 1, z1 + 1))
        reg(f"z_rear_cover_{i + 1}", "x_carriage", "shell", rc, orient="on its back (outer face down)", fill=1.0, group="gantry",
            extra=dict(note="hides T8x2 lead screw + Z cable chain; 3 segments"))

    # ---- Z carriage shroud + platform (z_carriage frame: origin = platform top, column centre at y=-32)
    yc_col = -(P.Z_PROFILE[1] / 2 + 12)
    zsh = rbox(-36, 36, yc_col - 36, 0, -132, 0, 6)
    zsh = zsh.cut(box(-33.6, 33.6, yc_col - 33.6, -2.4, -140, -2.4))  # 2.4 mm shell over wheels + plate
    zsh = zsh.cut(box(-27, 27, yc_col - 33.5, yc_col + 22.5, -140, 10))  # column + covers pass through
    reg("z_carriage_shroud", "z_carriage", "shell", zsh, orient="top face down", fill=1.0, group="gantry",
        extra=dict(note="covers 8 double-row V-wheels (2 rows x 4) and the T8 anti-backlash nut"))
    off = P.PLATFORM["offset"]
    pl = (
        cq.Workplane("XY")
        .moveTo(-40, -6)
        .lineTo(40, -6)
        .lineTo(52, off - 40)
        .threePointArc((0, off + 64), (-52, off - 40))
        .close()
        .extrude(16)
        .translate((0, 0, -16))
    )
    pl = fil(pl, 4, ">Z")
    pl = fil(pl, 2, "<Z")
    ribs = None
    for x in (-24, 24):
        r = cq.Workplane("YZ").polyline([(-6, -16), (off - 10, -16), (-6, -76)]).close().extrude(8).translate((x - 4, 0, 0))
        ribs = r if ribs is None else ribs.union(r)
    pl = pl.union(ribs)
    pl = pl.cut(cyl(7, (0, off, -17), (0, off, 1)))
    reg("lift_platform", "z_carriage", "shell", pl, orient="top face down, ribs up, 5 walls 40% gyroid", fill=0.55, group="gantry",
        extra=dict(note="option: 4 mm 6061 plate sandwich if long-term creep shows"))


def build_head():
    """Stereo-camera head with LED-ring "eyes" on the Z column top (rides on the X carriage)."""
    hg = P.HEAD_GEOM
    w, dp, h = hg["width"], hg["depth"], hg["height"]
    shell = fil(box(-w / 2, w / 2, -dp / 2, dp / 2, -h / 2, h / 2), 15.0)
    shell = shell.cut(fil(box(-w / 2 + WALL, w / 2 - WALL, -dp / 2 + WALL, dp / 2 - WALL, -h / 2 + WALL, h / 2 - WALL), 12.0))
    visor = None
    lenses = None
    rings = None
    for sx in (-1, 1):
        x = sx * hg["eye_dx"]
        shell = shell.cut(cyl(hg["ring_r"][1] + 1.0, (x, dp / 2 - 3.0, 0), (x, dp / 2 + 1, 0)))
        bezel = cyl(hg["ring_r"][1] + 1.0, (x, dp / 2 - 3.0, 0), (x, dp / 2 - 1.2, 0)).cut(cyl(hg["ring_r"][1], (x, dp / 2 - 4, 0), (x, dp / 2, 0)))
        visor = bezel if visor is None else visor.union(bezel)
        ln = cyl(hg["lens_r"], (x, dp / 2 - 3.0, 0), (x, dp / 2 - 0.8, 0))
        lenses = ln if lenses is None else lenses.union(ln)
        rg = cyl(hg["ring_r"][1], (x, dp / 2 - 3.0, 0), (x, dp / 2 - 1.6, 0)).cut(cyl(hg["ring_r"][0], (x, dp / 2 - 4, 0), (x, dp / 2, 0)))
        rings = rg if rings is None else rings.union(rg)
    def place(wp):
        return wp.rotate((0, 0, 0), (1, 0, 0), -hg["tilt_deg"]).translate(hg["center"])
    reg("head_shell", "x_carriage", "shell", place(shell), orient="split on the XY mid-plane, outer faces down", fill=1.0, group="gantry",
        extra=dict(note="houses a USB stereo camera board (60 mm baseline) + 2x 12-LED WS2812 rings; split on the mid-plane"))
    reg("head_eye_bezels", "x_carriage", "cap", place(visor), orient="flat", fill=1.0, group="gantry")
    reg("head_lenses", "x_carriage", "lens", place(lenses), printed=False, material="stereo camera lenses", group="gantry")
    reg("head_led_rings", "x_carriage", "accent", place(rings), printed=False, material="2x WS2812 12-LED ring (diffused)", group="gantry")
    cx, cy, cz = hg["center"]
    neck = fil(cyl(11, (cx, cy - 6, 10 + P.Z_COLUMN_LEN + 20), (cx, cy - 6, cz - 12)), 2, "%CIRCLE")
    reg("head_neck", "x_carriage", "cap", neck, orient="upright", fill=1.0, group="gantry")


def build_all():
    PARTS.clear()
    build_arm()
    # the MuJoCo "tool" body already sits on the roll axis (y = wrist_center_y), so tool parts use y = 0
    coupler_robot(0.0)
    build_tools(0.0)
    build_gantry()
    if P.HEAD:
        build_head()
    return PARTS
