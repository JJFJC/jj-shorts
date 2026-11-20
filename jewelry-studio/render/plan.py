"""Labelled top-down plan of the jewelry studio -> render/out/studio_plan.png"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mp  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parents[1]
L = json.loads((HERE / "cad" / "out" / "layout.json").read_text())
INK, MUTED, GRID = "#1f1f1e", "#6b6a66", "#d9d7d2"
ROBOT, HOT, HUMAN, PRINT = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"

# footprint (w, d) in mm, group, label
ST = {
    "pickle": ((170, 170), ROBOT, "Pickle pot\n(citric)"),
    "ultrasonic": ((170, 120), ROBOT, "Ultrasonic\ncleaner"),
    "tumbler": ((190, 190), ROBOT, "Vibratory\ntumbler"),
    "inspection": ((120, 120), ROBOT, "Inspection cam\n+ turntable"),
    "scale": ((110, 150), ROBOT, "Precision\nscale"),
    "polisher": ((320, 170), ROBOT, "Bench polisher\n(part-to-wheel)"),
    "laser": ((300, 280), ROBOT, "Laser\nengraver"),
    "invest": ((220, 220), HOT, "Vacuum investing\n+ wax tree"),
    "kiln": ((300, 300), HOT, "Burnout kiln"),
    "furnace": ((160, 160), HOT, "Melting\nfurnace"),
    "caster": ((220, 220), HOT, "Vacuum\ncaster"),
    "torch": ((320, 260), HOT, "FIXED torch station\n+ fume hood"),
    "bench_pin": ((200, 340), HUMAN, "Hand bench:\nbench pin, mandrel"),
}
EXTRA = [
    ((-1029, 106), 389, 388, PRINT, "Bambu P1S\n(wax/resin patterns)"),
    ((220, 245), 260, 75, ROBOT, "Tool rack\n(3 spindles)"),
    ((-65, 165), 130, 80, ROBOT, "Servo vise"),
    ((-940, 585), 160, 110, HUMAN, "Finished\ntray"),
]

fig, ax = plt.subplots(figsize=(14, 8.4), dpi=170)
ax.add_patch(mp.Rectangle((-1350, -720), 2700, 1480, fc="#f6f4ef", ec=GRID, lw=1))
# reach: guaranteed table-height area and the raised-tool band
ax.add_patch(mp.Rectangle((-450, 110), 900, 190, fc=ROBOT, alpha=0.10, ec=ROBOT, lw=1, ls="--"))
ax.text(-440, 118, "arm work area at table height (900 × 190 mm, guaranteed)", fontsize=7.5, color=ROBOT, va="bottom")
ax.add_patch(mp.Rectangle((-650, 300), 1600, 250, fc=ROBOT, alpha=0.05, ec=ROBOT, lw=0.8, ls=":"))
ax.text(-90, 545, "raised reach band: stations served from above (concept, verify with IK)", fontsize=7.5, color=ROBOT, va="top")
# rail + wall
ax.add_patch(mp.Rectangle((-500, -30), 1000, 60, fc=INK, ec=INK))
ax.text(0, -55, "X rail (2060 V-slot) + Z gantry", ha="center", va="top", fontsize=8, color=INK)
ax.add_patch(mp.Rectangle((-700, -250), 1800, 8, fc=HOT, ec=HOT))
ax.text(1110, -246, "steel heat-shield wall\n(arm cannot reach past it)", fontsize=8, color=HOT, va="center")
ax.add_patch(mp.Rectangle((-670, -160), 86, 120, fc=MUTED, ec=MUTED))
ax.text(-627, -170, "controller\n+ e-stop", ha="center", va="top", fontsize=7, color=MUTED)
for k, ((w, d), c, lab) in ST.items():
    x, y = L[k]
    if k == "bench_pin":
        x, y = x + 50, y + 20
    ax.add_patch(mp.FancyBboxPatch((x - w / 2, y - d / 2), w, d, boxstyle="round,pad=0,rounding_size=12", fc=c, alpha=0.18, ec=c, lw=1.3))
    ax.text(x, y, lab, ha="center", va="center", fontsize=7.5, color=INK)
for (x0, y0), w, d, c, lab in EXTRA:
    ax.add_patch(mp.FancyBboxPatch((x0, y0), w, d, boxstyle="round,pad=0,rounding_size=10", fc=c, alpha=0.18, ec=c, lw=1.3))
    ax.text(x0 + w / 2, y0 + d / 2, lab, ha="center", va="center", fontsize=7.5, color=INK)
ax.add_patch(mp.Circle((1000, -330), 50, fc=HOT, alpha=0.5, ec=HOT))
ax.text(1000, -395, "extinguisher", ha="center", va="top", fontsize=7, color=INK)
ax.text(-1320, -690, "HOT ZONE: person only", fontsize=11, color=HOT, weight="bold")
ax.text(-560, 720, "COLD WORK: robot-served row", fontsize=11, color=ROBOT, weight="bold", va="top")
handles = [mp.Patch(fc=c, alpha=0.4, ec=c, label=t) for c, t in ((ROBOT, "robot-served"), (HOT, "hot work (human)"), (HUMAN, "hand bench (human)"), (PRINT, "printing"))]
ax.legend(handles=handles, loc="lower right", frameon=False, fontsize=8)
ax.set_xlim(-1400, 1400)
ax.set_ylim(-760, 780)
ax.set_aspect("equal")
ax.set_xlabel("x (mm, along the rail)", color=MUTED, fontsize=8)
ax.set_ylabel("y (mm, + = toward the operator)", color=MUTED, fontsize=8)
ax.tick_params(colors=MUTED, labelsize=7)
for s in ax.spines.values():
    s.set_visible(False)
ax.set_title("Jewelry studio around the Gantry Arm: plan view (bench 2.7 × 1.48 m)", color=INK, fontsize=12, loc="left")
fig.tight_layout()
out = HERE / "render" / "out" / "studio_plan.png"
fig.savefig(out, facecolor="white")
print("wrote", out)
