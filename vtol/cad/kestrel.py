"""KESTREL airframe CAD (CadQuery) - original design, all parts from params.py.

    python cad/kestrel.py            -> out/<mode>/*.stl for mode in hover, cruise (+ STEP, manifest)

Shapes: NACA 00xx sections lofted into a smooth blended wing-body, an elliptical nose snout
carrying the canard, swept winglets, and 4 tilt-rotor pods (canard-tip pullers, trailing-edge
pushers). Printed pieces are split so each fits the 256 mm bed.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cadquery as cq
from cadquery import Vector as V

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
import params as P  # noqa: E402

OUT = ROOT / "cad" / "out"
BED = 256.0 - 12.0


# ---------------------------------------------------------------- primitives
def naca00(c: float, t: float, n: int = 36):
    """Upper TE->LE then lower LE->TE, (x, z) in mm, closed by a short TE line."""
    xs = [0.5 * (1 - math.cos(math.pi * i / n)) for i in range(n + 1)]
    zt = [5 * t * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4) for x in xs]
    upper = [(x * c, z * c) for x, z in zip(xs, zt)][::-1]
    lower = [(x * c, -z * c) for x, z in zip(xs, zt)][1:]
    return upper + lower


def section(y: float, c: float, x_le: float, t: float, twist_deg: float = 0.0, z0: float = 0.0, axis: str = "y") -> cq.Wire:
    """Airfoil wire at span station y (axis='y') or at height z for vertical fins (axis='z')."""
    pts = []
    tw = math.radians(twist_deg)
    for x, z in naca00(c, t):
        xc = x - 0.25 * c  # twist about the quarter chord
        xr = xc * math.cos(tw) + z * math.sin(tw) + 0.25 * c
        zr = -xc * math.sin(tw) + z * math.cos(tw)
        if axis == "y":
            pts.append(V(x_le + xr, y, z0 + zr))
        else:  # fin: airfoil thickness along y, section plane at height y-arg
            pts.append(V(x_le + xr, zr, y))
    spline = cq.Edge.makeSpline(pts)
    te = cq.Edge.makeLine(pts[-1], pts[0])
    return cq.Wire.assembleEdges([spline, te])


def W(s) -> cq.Workplane:
    return cq.Workplane("XY").add(s)


def box(x0, x1, y0, y1, z0, z1):
    return W(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0)))


def cyl(r, p0, p1):
    d = V(*p1) - V(*p0)
    return W(cq.Solid.makeCylinder(r, d.Length, V(*p0), d.normalized()))


def ellipse_wire(x, w, h, z=0.0):
    return cq.Wire.makeEllipse(w / 2, h / 2, V(x, 0, z), V(1, 0, 0), V(0, 1, 0))


def rot(wp, axis_pt, axis_dir, deg):
    return wp.rotate(axis_pt, (axis_pt[0] + axis_dir[0], axis_pt[1] + axis_dir[1], axis_pt[2] + axis_dir[2]), deg)


# ---------------------------------------------------------------- airframe
def _hermite(ys, vals, y):
    """Catmull-Rom through (ys, vals), symmetric about y=0 (zero slope at the centreline)."""
    for i in range(len(ys) - 1):
        if ys[i] <= y <= ys[i + 1]:
            h = ys[i + 1] - ys[i]
            t = (y - ys[i]) / h
            m0 = 0.0 if i == 0 else (vals[i + 1] - vals[i - 1]) / (ys[i + 1] - ys[i - 1]) * h
            m1 = (vals[i + 1] - vals[i]) if i + 2 >= len(ys) else (vals[i + 2] - vals[i]) / (ys[i + 2] - ys[i]) * h
            h00, h10, h01, h11 = 2 * t**3 - 3 * t**2 + 1, t**3 - 2 * t**2 + t, -2 * t**3 + 3 * t**2, t**3 - t**2
            return h00 * vals[i] + h10 * m0 + h01 * vals[i + 1] + h11 * m1
    return vals[-1]


def wing_station(y: float):
    """(chord, x_le, thickness, twist) - smooth through the body blend, linear taper outboard."""
    ya = abs(y)
    yb, tip = P.SECTIONS[2][0], P.SECTIONS[-1][0]
    if ya <= yb:
        ys = [s[0] for s in P.SECTIONS[:3]] + [P.SECTIONS[3][0]]
        cols = list(zip(*[(s[1], s[2], s[3]) for s in P.SECTIONS]))
        c, xle, t = (_hermite(ys, list(col), ya) for col in cols)
        return c, xle, t, 0.0
    c, xle, t = P.chord_at(ya)
    return c, xle, t, -P.WASHOUT * (ya - yb) / (tip - yb)


def main_wing() -> cq.Workplane:
    yb, tip = P.SECTIONS[2][0], P.SECTIONS[-1][0]
    half = [i * 10.0 for i in range(int(yb / 10) + 1)] + [yb + (tip - yb) * k / 4 for k in range(1, 5)]
    ys = sorted({-y for y in half} | set(half))
    wires = [section(y, *wing_station(y)[:3], wing_station(y)[3]) for y in ys]
    return W(cq.Solid.makeLoft(wires, True))


def nose() -> cq.Workplane:
    N = P.NOSE
    xs = [N["x_tip"], N["x_tip"] + 25, N["x_tip"] + 90, N["x_tip"] + 190, N["x_join"]]
    ws = [10, 52, N["width"] * 0.92, N["width"], N["width"] * 0.96]
    hs = [10, 40, N["height"] * 0.9, N["height"], N["height"] * 0.92]
    wires = [ellipse_wire(x, w, h, z=4.0) for x, w, h in zip(xs, ws, hs)]
    return W(cq.Solid.makeLoft(wires, False))


def canard() -> cq.Workplane:
    C = P.CANARD
    half = C["span"] / 2
    wires = [section(y, C["chord"], C["x_le"], C["thickness"], C["incidence"], z0=C["z"]) for y in (-half, 0.0, half)]
    return W(cq.Solid.makeLoft(wires, True))


def winglet(sgn: int) -> cq.Workplane:
    Wl = P.WINGLET
    y_tip, c_tip, x_tip, _ = P.SECTIONS[-1]
    x0 = x_tip + c_tip - Wl["root_chord"]
    x1 = x0 + Wl["height"] * math.tan(math.radians(Wl["sweep"])) + (Wl["root_chord"] - Wl["tip_chord"])
    w0 = section(0.0, Wl["root_chord"], x0, 0.08, axis="z")
    w1 = section(Wl["height"], Wl["tip_chord"], x1, 0.08, axis="z")
    fin = W(cq.Solid.makeLoft([w0, w1], True))
    return fin.translate((0, sgn * (y_tip - 1.0), -2.0))


# ---------------------------------------------------------------- rotor pods
PROP_R = P.PROP_D_IN * 25.4 / 2


def prop(n_blades=2) -> cq.Workplane:
    """Thin twisted-look blades in the YZ plane (thrust axis X), hub at origin."""
    out = cyl(6.0, (-6, 0, 0), (4, 0, 0))
    for k in range(n_blades):
        blade = box(-1.2, 1.2, 5.0, PROP_R, -7.0, 7.0).edges("|X").fillet(1.0)
        blade = rot(blade, (0, 0, 0), (0, 1, 0), 12.0)  # pitch
        out = out.union(rot(blade, (0, 0, 0), (1, 0, 0), k * 360.0 / n_blades))
    return out


def tilt_unit(pusher: bool):
    """(housing, motor, prop) in the pivot frame with the thrust/shaft axis along X.

    puller: prop at -X ahead of the motor; pusher: prop at +X behind it.
    """
    s = 1 if pusher else -1
    housing = cyl(15.0, (0, 0, 0), (s * 30.0, 0, 0)).union(cyl(9.0, (0, -16, 0), (0, 16, 0)))  # bell + pivot axle boss
    motor = cyl(14.0, (s * 30.0, 0, 0), (s * 48.0, 0, 0)).union(cyl(2.5, (s * 48.0, 0, 0), (s * 58.0, 0, 0)))
    pr = prop().translate((s * 56.0, 0, 0))
    return housing, motor, pr


def place_tilt(parts, pivot, tilt_deg, pusher):
    """tilt 0 = cruise; 90 = hover (thrust up). Puller axis -X -> +Z, pusher shaft +X -> +Z."""
    ang = tilt_deg if not pusher else -tilt_deg
    return [rot(p, (0, 0, 0), (0, 1, 0), ang).translate(pivot) for p in parts]


def pod(x0, x1, y, z, r=16.0):
    body = cq.Workplane("YZ").workplane(offset=x0).center(y, z).circle(r).extrude(x1 - x0)
    return body.edges().fillet(r * 0.9)


# ---------------------------------------------------------------- assembly / parts
def build(tilt_deg: float):
    parts = []  # (name, role, shape, printed)

    def add(name, role, shape, printed=True):
        parts.append((name, role, shape, printed))

    wing = main_wing()
    # printable split: body (|y|<130) into 4, outer wing into 2 panels per side
    yb = P.SECTIONS[2][0]
    ymid = 0.5 * (yb + P.SECTIONS[-1][0])
    big = 2000
    x_cut = 90.0
    for side, sg in (("L", -1), ("R", 1)):
        ylo, yhi = (0, yb) if sg > 0 else (-yb, 0)
        add(f"body_front_{side}", "shell", wing.intersect(box(-big, x_cut, ylo, yhi, -big, big)))
        add(f"body_rear_{side}", "shell", wing.intersect(box(x_cut, big, ylo, yhi, -big, big)))
        a, b = sorted((sg * yb, sg * ymid))
        add(f"wing_inner_{side}", "shell", wing.intersect(box(-big, big, a, b, -big, big)))
        a, b = sorted((sg * ymid, sg * big))
        add(f"wing_outer_{side}", "shell", wing.intersect(box(-big, big, a, b, -big, big)))
        add(f"winglet_{side}", "accent2", winglet(sg))
    nz = nose()
    add("nose_front", "shell", nz.intersect(box(-big, -225.0, -big, big, -big, big)))
    add("nose_rear", "shell", nz.intersect(box(-225.0, big, -big, big, -big, big)))
    cn = canard()
    add("canard_L", "shell", cn.intersect(box(-big, big, -big, 0, -big, big)))
    add("canard_R", "shell", cn.intersect(box(-big, big, 0, big, -big, big)))
    add("nose_camera", "lens", cyl(9.0, (P.NOSE["x_tip"] - 1.0, 0, 4.0), (P.NOSE["x_tip"] + 6.0, 0, 4.0)), printed=False)

    # front pullers at the canard tips
    F = P.FRONT_ROTOR
    for side, sg in (("L", -1), ("R", 1)):
        y = sg * F["y"]
        add(f"front_pod_{side}", "cap", pod(F["x_pivot"] + 8, F["x_pivot"] + 95, y, F["z"]))
        h, m, pr = place_tilt(tilt_unit(False), (F["x_pivot"], y, F["z"]), tilt_deg, pusher=False)
        add(f"front_tilt_{side}", "cap", h)
        add(f"front_motor_{side}", "motor", m, printed=False)
        add(f"front_prop_{side}", "prop", pr, printed=False)
    # rear pushers behind the main-wing trailing edge
    R = P.REAR_ROTOR
    for side, sg in (("L", -1), ("R", 1)):
        y = sg * R["y"]
        c, xle, _ = P.chord_at(R["y"])
        te = xle + c
        x_piv = te + R["behind_te"]
        add(f"rear_pod_{side}", "cap", pod(xle + 0.35 * c, x_piv - 8, y, R["z"]))
        # landing fin under the rear pod
        fin = box(x_piv - 70, x_piv - 20, y - 2, y + 2, R["z"] - 60, R["z"]).edges("|Y").fillet(1.5)
        add(f"rear_skid_{side}", "cap", fin)
        h, m, pr = place_tilt(tilt_unit(True), (x_piv, y, R["z"]), tilt_deg, pusher=True)
        add(f"rear_tilt_{side}", "cap", h)
        add(f"rear_motor_{side}", "motor", m, printed=False)
        add(f"rear_prop_{side}", "prop", pr, printed=False)
    # nav lights: red left, green right winglet tips
    y_tip = P.SECTIONS[-1][0]
    x_light = P.SECTIONS[-1][2] + P.SECTIONS[-1][1] * 0.5
    add("nav_left", "nav_red", cyl(4.0, (x_light, -y_tip - 2.0, 0), (x_light, -y_tip + 1.0, 0)), printed=False)
    add("nav_right", "nav_green", cyl(4.0, (x_light, y_tip - 1.0, 0), (x_light, y_tip + 2.0, 0)), printed=False)
    return parts


def fits_bed(dims) -> bool:
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


def main():
    report = []
    for mode, tilt in (("hover", 90.0), ("cruise", 0.0)):
        out = OUT / mode
        out.mkdir(parents=True, exist_ok=True)
        manifest = []
        for name, role, shape, printed in build(tilt):
            s = shape.val() if shape.size() == 1 else shape.combine().val()
            bb = s.BoundingBox()
            dims = [bb.xlen, bb.ylen, bb.zlen]
            cq.exporters.export(shape, str(out / f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
            manifest.append(dict(name=name, role=role, printed=printed, valid=s.isValid(), dims=[round(d, 1) for d in dims],
                                 fits_bed=fits_bed(dims) if printed else None, volume_cm3=round(s.Volume() / 1000, 1)))
        (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
        report.append((mode, manifest))
    for mode, man in report[:1]:
        bad = [m["name"] for m in man if m["printed"] and not m["fits_bed"]]
        inval = [m["name"] for m in man if not m["valid"]]
        print(f"{len(man)} parts; printed {sum(m['printed'] for m in man)}; bed misfits {bad}; invalid {inval}")


if __name__ == "__main__":
    main()
