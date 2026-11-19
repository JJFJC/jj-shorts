"""Fit-test coupon: print this FIRST on the new printer, before any real part.

Every hole the design relies on, in three sizes (tight / nominal / loose), plus a snap clip
and the LED channel. Note which size fits each item, then set the matching tolerances in
params.py before printing real parts.

    python cad/test_prints.py   ->  cad/out/test_prints/fit_coupon.stl (+ .step)

Print: PETG, 0.2 mm layers, 6 walls, flat on the textured plate, no supports (~35 min, ~20 g).
"""

import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from parts import box, cyl  # noqa: E402

OUT = HERE / "out" / "test_prints"

# (label, diameters tight/nominal/loose, depth, what it tests)
ROWS = [
    ("INS", (3.8, 4.0, 4.2), 5.0, "M3x4 heat-set insert (melt in; want firm, no cracking)"),
    ("M3", (3.2, 3.4, 3.6), 6.0, "M3 screw clearance (want free slide)"),
    ("MAG", (8.0, 8.2, 8.4), 3.2, "8x3 mm N52 magnet (want press fit by hand)"),
    ("POGO", (2.0, 2.2, 2.4), 6.0, "2 mm pogo pin (want snug slide, then glue)"),
    ("PIN", (2.9, 3.1, 3.3), 6.0, "3 mm steel dowel, gripper pivots (want press fit)"),
]
W, H, T = 96.0, 74.0, 6.0


def coupon() -> cq.Workplane:
    c = box(0, W, 0, H, 0, T).edges("|Z").fillet(3)
    for r, (label, dias, depth, _) in enumerate(ROWS):
        y = H - 10 - r * 12.5
        c = c.cut(cq.Workplane("XY").text(label, 4.5, -0.6, halign="left", valign="center").translate((3, y, T)))
        for k, d in enumerate(dias):
            x = 34 + k * 14
            c = c.cut(cyl(d / 2, (x, y, T - depth), (x, y, T + 1)))
    for k, lab in enumerate(("TIGHT", "NOM", "LOOSE")):
        c = c.cut(cq.Workplane("XY").text(lab, 2.6, -0.4).translate((34 + k * 14, 3.5, T)))
    # LED channel exactly as on the upper arm (5 mm wide, 1.6 deep): lay your LED strip in it
    c = c.cut(box(84, 89.0, 8, H - 6, T - 1.6, T + 1))
    return c


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name, shape in (("fit_coupon", coupon()),):
        cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(shape, str(OUT / f"{name}.step"))
    print("wrote", sorted(p.name for p in OUT.iterdir()))


if __name__ == "__main__":
    main()
