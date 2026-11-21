"""Candle cell CAD (CadQuery): printable mold set + carousel concept assembly.

    python cad/cell.py      -> cad/out/mold/*.stl (print these) and cad/out/cell/*.stl (concept)

Mold = two halves split on a plane through the axis, printed flat (layer lines run along the
axis, so the candle slides UP out of the open top) and screwed together for good; a loose floor
piston carries the wick tab and is pushed up from below to eject; a wick bar with a side-open
slot centres the wick while pouring.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cadquery as cq
from cadquery import Vector as V

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import params as P  # noqa: E402

OUT = ROOT / "cad" / "out"
BED = 256.0 - 12.0


def W(s):
    return cq.Workplane("XY").add(s)


def box(x0, x1, y0, y1, z0, z1):
    return W(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0)))


def cyl(r, p0, p1):
    d = V(*p1) - V(*p0)
    return W(cq.Solid.makeCylinder(r, d.Length, V(*p0), d.normalized()))


def cone(r0, r1, p0, p1):
    d = V(*p1) - V(*p0)
    return W(cq.Solid.makeCone(r0, r1, d.Length, V(*p0), d.normalized()))


def fil(wp, r, sel=None):
    try:
        return (wp.edges(sel) if sel else wp.edges()).fillet(r)
    except Exception:
        return wp


# ---------------------------------------------------------------- mold (local frame: axis z, floor at z=0)
C, M = P.CANDLE, P.MOLD
R = C["d"] / 2
H_CAV = C["h"] + M["rim"]
R_TOP = R + H_CAV * math.tan(math.radians(C["draft_deg"]))
HALF_W = R_TOP + M["wall"]
H_MOLD = M["floor"] + H_CAV
PISTON_R = R - 0.3  # piston = the cavity floor; slides up through the bore


def mold_body():
    body = fil(box(-HALF_W, HALF_W, -HALF_W, HALF_W, 0, H_MOLD), 8.0, "|Z")
    body = body.cut(cone(R, R_TOP, (0, 0, M["floor"]), (0, 0, H_MOLD + 0.01)))
    body = body.cut(cyl(R, (0, 0, -1), (0, 0, M["floor"] + 0.01)))  # piston bore through the floor
    body = body.cut(cone(R_TOP, R_TOP + 3, (0, 0, H_MOLD - 3), (0, 0, H_MOLD + 0.01)))  # pour chamfer
    # flange feet with M4 holes to bolt to the carousel
    for sx in (-1, 1):
        foot = fil(box(sx * HALF_W - (12 if sx > 0 else -12) * 0 - (0 if sx > 0 else 14), sx * HALF_W + (14 if sx > 0 else 0), -14, 14, 0, 6), 3, "|Z")
        body = body.union(foot)
        body = body.cut(cyl(2.2, (sx * (HALF_W + 7), 0, -1), (sx * (HALF_W + 7), 0, 7)))
    return body


def mold_halves():
    """Split on the YZ plane; 3x M3 joining screws per side through bosses, + 2 alignment pins."""
    body = mold_body()
    big = 500
    halves = []
    for sx in (-1, 1):
        h = body.intersect(box(min(0, sx * big), max(0, sx * big), -big, big, -big, big))
        for zz in (14, H_MOLD / 2, H_MOLD - 14):
            for sy in (-1, 1):
                y = sy * (HALF_W - 4.5)
                h = h.cut(cyl(1.7, (-big, y, zz), (big, y, zz)))  # M3 through-holes (nut trap on the other side by slicer modifier)
        halves.append(h)
    return halves


def piston():
    p = cyl(PISTON_R, (0, 0, 0), (0, 0, M["floor"]))
    p = p.cut(cyl(M["tab_d"] / 2, (0, 0, M["floor"] - M["tab_depth"]), (0, 0, M["floor"] + 0.1)))  # wick tab seat
    p = p.cut(cyl(4.0, (0, 0, -0.1), (0, 0, 2.0)))  # ejector pin centring dimple
    return fil(p, 0.6, "%CIRCLE")


def wick_bar():
    L = 2 * HALF_W
    bar = fil(box(-L / 2, L / 2, -6, 6, H_MOLD, H_MOLD + 5), 2.0, "|Z")
    bar = bar.cut(box(-1.2, 1.2, -7, 0.0, H_MOLD - 1, H_MOLD + 6))  # side-open wick slot (opens to -y)
    for sx in (-1, 1):  # locating lugs that drop into the pour chamfer
        bar = bar.union(box(sx * (R_TOP - 1.5) - 1.5, sx * (R_TOP - 1.5) + 1.5, -4, 4, H_MOLD - 3, H_MOLD))
    return bar


def candle_solid(fill=1.0):
    h = C["h"] * fill
    r_h = R + h * math.tan(math.radians(C["draft_deg"]))
    return cone(R, r_h, (0, 0, M["floor"]), (0, 0, M["floor"] + h))


# ---------------------------------------------------------------- carousel concept
def at_station(wp, k, z=0.0, radial_rot=True):
    x, y = P.station_xy(k)
    ang = -90.0 + k * 360.0 / P.N_STATIONS
    w = wp.rotate((0, 0, 0), (0, 0, 1), ang + 90.0) if radial_rot else wp
    return w.translate((x, y, z))


def build_cell():
    parts = []

    def add(name, role, shape):
        parts.append((name, role, shape))

    D = P.DISC
    z_top = D["z"]
    add("base", "frame", fil(box(-360, 360, -380, 360, 0, 18), 10, "|Z"))
    add("column", "frame", cyl(28, (0, 0, 18), (0, 0, P.POT["z"])))
    disc = cyl(D["d"] / 2, (0, 0, z_top - D["t"]), (0, 0, z_top)).cut(cyl(34, (0, 0, 0), (0, 0, 200)))
    for k in range(P.N_STATIONS):
        x, y = P.station_xy(k)
        disc = disc.cut(cyl(7, (x, y, 0), (x, y, 200)))  # ejector pin holes
    add("carousel_disc", "disc", disc)
    add("drive_motor", "dark", box(-21, 21, 70, 112, 18, 66))
    add("drive_belt_ring", "dark", cyl(D["d"] / 2 - 4, (0, 0, z_top - D["t"] - 6), (0, 0, z_top - D["t"])).cut(cyl(D["d"] / 2 - 12, (0, 0, 0), (0, 0, 200))))
    for k in range(P.N_STATIONS):
        job = P.STATIONS[k]
        halves = mold_halves()
        add(f"mold{k}_a", "mold", at_station(halves[0], k, z_top))
        add(f"mold{k}_b", "mold", at_station(halves[1], k, z_top))
        add(f"piston{k}", "mold2", at_station(piston(), k, z_top))
        if job in ("cool", "top-off pour") or k == 1:
            fill = 0.55 if k == 1 else 0.97
            add(f"wax{k}", "wax_liquid" if k in (1, 2, 3) else "wax", at_station(candle_solid(fill), k, z_top))
            add(f"wickbar{k}", "mold2", at_station(wick_bar(), k, z_top))
            add(f"wick{k}", "dark", at_station(cyl(0.9, (0, 0, M["floor"]), (0, 0, H_MOLD + 10)), k, z_top))
        if k == 0:
            add("wickbar0", "mold2", at_station(wick_bar(), 0, z_top))
            add("wick0", "dark", at_station(cyl(0.9, (0, 0, M["floor"] - 1), (0, 0, H_MOLD + 10)), 0, z_top))
    # heated pot + two pour spouts (stations 1 and 7)
    pot = fil(cyl(P.POT["d"] / 2, (0, 0, P.POT["z"]), (0, 0, P.POT["z"] + P.POT["h"])), 6, "%CIRCLE")
    add("wax_pot", "steel", pot)
    add("pot_lid", "frame", fil(cyl(P.POT["d"] / 2 + 4, (0, 0, P.POT["z"] + P.POT["h"]), (0, 0, P.POT["z"] + P.POT["h"] + 12)), 4, "%CIRCLE"))
    rim_z = z_top + H_MOLD
    for k in (1, 7):
        x, y = P.station_xy(k)
        d = math.hypot(x, y)
        ux, uy = x / d, y / d
        p0 = (ux * P.POT["d"] / 2 * 0.8, uy * P.POT["d"] / 2 * 0.8, P.POT["z"] + 20)
        p1 = (x * 0.97, y * 0.97, rim_z + 35)
        add(f"spout{k}", "steel", cyl(7, p0, p1).union(cyl(5, p1, (x, y, rim_z + 12))))
        add(f"valve{k}", "dark", box(-12, 12, -10, 10, -10, 10).translate((x * 0.97, y * 0.97, rim_z + 40)))
    # cooling tunnel over stations 2..6 and 8..9 (fans on top)
    for k0, k1 in ((2, 6), (8, 9)):
        a0 = math.radians(-90 + (k0 - 0.45) * 30)
        a1 = math.radians(-90 + (k1 + 0.45) * 30)
        pts_out = [(320 * math.cos(a0 + (a1 - a0) * i / 20), 320 * math.sin(a0 + (a1 - a0) * i / 20)) for i in range(21)]
        pts_in = [(150 * math.cos(a1 - (a1 - a0) * i / 20), 150 * math.sin(a1 - (a1 - a0) * i / 20)) for i in range(21)]
        roof = cq.Workplane("XY").polyline(pts_out + pts_in).close().extrude(4).translate((0, 0, rim_z + 45))
        add(f"tunnel_roof_{k0}", "cover", roof)
        for k in range(k0, k1 + 1, 2):
            x, y = P.station_xy(k)
            add(f"fan_{k}", "dark", box(-60, 60, -60, 60, 0, 25).translate((x, y, rim_z + 49)))
    # demold station 10: ejector (lead screw under the disc) + sweeper arm
    x, y = P.station_xy(10)
    add("ejector_motor", "dark", box(-21, 21, -21, 21, 18, 66).translate((x, y, 0)))
    add("ejector_screw", "steel", cyl(4, (x, y, 66), (x, y, z_top - D["t"] - 2)))
    add("sweeper_servo", "dark", box(-12, 12, -20, 20, 0, 40).translate((x * 0.62, y * 0.62, rim_z - 10)))
    add("sweeper_arm", "frame", box(0, 95, -5, 5, 0, 8).rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(y, x)) + 25).translate((x * 0.62, y * 0.62, rim_z + 30)))
    # chute -> finishing turntable (heat-gun gloss + wick trim) -> output tray
    ux, uy = x / math.hypot(x, y), y / math.hypot(x, y)
    chute = box(0, 120, -38, 38, 0, 3).rotate((0, 0, 0), (0, 1, 0), 18).rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(y, x)))
    add("chute", "cover", chute.translate((ux * 300, uy * 300, rim_z - 8)))
    fx, fy = ux * 470, uy * 470
    add("finish_turntable", "frame", cyl(45, (fx, fy, 18), (fx, fy, 70)))
    add("finished_candle", "wax", candle_solid(1.0).rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, C["h"] + 2 * M["floor"])).translate((fx, fy, 70 - M["floor"])))
    add("heatgun", "accent", box(-25, 25, -25, 25, 0, 130).rotate((0, 0, 0), (0, 1, 0), 60).translate((fx - 90 * ux + 10, fy - 90 * uy, 210)))
    add("wick_trimmer", "dark", box(-10, 10, -10, 10, 0, 50).translate((fx + 40 * uy, fy - 40 * ux, 170)))
    # output tray with finished candles
    tx, ty = -420.0, 120.0
    add("output_tray", "frame", fil(box(tx - 90, tx + 90, ty - 140, ty + 140, 0, 8), 6, "|Z"))
    for i in range(2):
        for j in range(3):
            add(f"out_candle_{i}{j}", "wax", candle_solid(1.0).translate((tx - 40 + 80 * i, ty - 90 + 90 * j, 8 - M["floor"])))
    add("controller", "frame", fil(box(240, 340, -370, -290, 18, 120), 8))
    add("estop", "estop_red", cyl(16, (290, -330, 120), (290, -330, 136)))
    return parts


def fits_bed(dims):
    for up in range(3):
        h = dims[up]
        a, b = [d for i, d in enumerate(dims) if i != up]
        if h > BED:
            continue
        for deg in range(0, 91):
            t = math.radians(deg)
            if a * math.cos(t) + b * math.sin(t) <= BED and a * math.sin(t) + b * math.cos(t) <= BED:
                return True
    return False


def export(parts, sub):
    out = OUT / sub
    out.mkdir(parents=True, exist_ok=True)
    man = []
    for name, role, shape in parts:
        s = shape.val() if shape.size() == 1 else shape.combine().val()
        bb = s.BoundingBox()
        cq.exporters.export(shape, str(out / f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        if sub == "mold":
            cq.exporters.export(shape, str(out / f"{name}.step"))
        man.append(dict(name=name, role=role, valid=s.isValid(), dims=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
                        volume_cm3=round(s.Volume() / 1000, 1), fits_bed=fits_bed([bb.xlen, bb.ylen, bb.zlen])))
    (out / "manifest.json").write_text(json.dumps(man, indent=1))
    return man


def main():
    a, b = mold_halves()
    mold = [("mold_half_a", "mold", a), ("mold_half_b", "mold", b), ("piston", "mold2", piston()), ("wick_bar", "mold2", wick_bar())]
    man = export(mold, "mold")
    for m in man:
        print(f"{m['name']:14s} {m['dims']} valid={m['valid']} fits={m['fits_bed']} {m['volume_cm3']} cm3")
    cell = export(build_cell(), "cell")
    print(len(cell), "cell parts; invalid:", [m["name"] for m in cell if not m["valid"]])


if __name__ == "__main__":
    main()
