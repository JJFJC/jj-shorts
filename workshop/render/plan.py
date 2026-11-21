"""Workshop plan: top view of the bench + cart, and the pegboard wall elevation -> render/out/workshop_plan.png"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mp  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parents[1]
L = json.loads((HERE / "cad" / "out" / "layout.json").read_text())
R = json.loads((HERE / "cad" / "out" / "reach.json").read_text())
INK, MUTED, GRID = "#1f1f1e", "#6b6a66", "#d9d7d2"
ROBOT, HOT, HUMAN, PRINT = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
B, C, PEG = L["bench"], L["cart"], L["peg"]
WALL = L["wall_y"]

fig, (ax, ax2) = plt.subplots(2, 1, figsize=(13, 11.5), dpi=170, gridspec_kw=dict(height_ratios=[1.0, 1.05]))


def rect(a, x0, y0, w, d, c, lab=None, alpha=0.18, fs=7.5, r=10):
    a.add_patch(mp.FancyBboxPatch((x0, y0), w, d, boxstyle=f"round,pad=0,rounding_size={r}", fc=c, alpha=alpha, ec=c, lw=1.2))
    if lab:
        a.text(x0 + w / 2, y0 + d / 2, lab, ha="center", va="center", fontsize=fs, color=INK)


# ---------------------------------------------------------------- top view
ax.add_patch(mp.Rectangle((-1800, WALL - 40), 3100, 40, fc=MUTED, ec=MUTED))
ax.text(-1780, WALL - 45, "WALL  (pegboard above the bench, steel heat panel above the cart)", fontsize=8, color=MUTED, va="top")
ax.add_patch(mp.Rectangle((B["x0"], B["y0"]), B["x1"] - B["x0"], B["y1"] - B["y0"], fc="#f6f4ef", ec=GRID, lw=1.2))
ax.text(B["x1"] - 10, B["y1"] + 15, "IKEA KARLBY top 1860 x 635 x 38", ha="right", va="bottom", fontsize=8, color=MUTED)
ax.add_patch(mp.Rectangle((-450, 110), 900, 190, fc=ROBOT, alpha=0.07, ec=ROBOT, lw=0.8, ls="--"))
ax.add_patch(mp.Rectangle((-500, -30), 1000, 60, fc=INK, ec=INK))
ax.text(0, 40, "X rail + Z gantry", ha="center", va="top", fontsize=7.5, color=INK)
rect(ax, -670, -160, 86, 120, MUTED, "ctrl", alpha=0.5, fs=6.5, r=4)
size = dict(vise=(130, 160), tool_rack=(260, 75), ultrasonic=(170, 120), tumbler=(190, 190), inspection=(120, 120), scale=(110, 150), pickle=(140, 140))
lab = dict(vise="Servo vise", tool_rack="Tool rack\n3 spindles", ultrasonic="Ultrasonic", tumbler="Tumbler", inspection="Camera +\nturntable", scale="Scale", pickle="Pickle")
for k, (w, d) in size.items():
    x, y = L["stations"][k]["xy"]
    if k == "tool_rack":
        x0, y0 = 220, 245
    elif k == "vise":
        x0, y0 = x - w / 2, 123
        d = 122
    else:
        x0, y0 = x - w / 2, y - d / 2
    rect(ax, x0, y0, w, d, ROBOT, lab[k] + ("" if R[k]["ok"] else "\n(NO REACH)"))
rect(ax, 511, 35, 389, 389, PRINT, "Bambu P1S\n(door faces the arm)")
rect(ax, -830, 60, 130, 100, HUMAN, "mat", fs=6.5)
rect(ax, -910, 0, 90, 120, HUMAN, "vise", fs=6.5)
cx, r, fy = -700, 150, B["y1"]
ax.add_patch(mp.Wedge((cx, fy), r, 180, 360, fc="white", ec=HUMAN, lw=1.5))
rect(ax, cx - 30, fy - r - 40, 60, 190, HUMAN, None, alpha=0.5, r=4)
ax.text(cx, fy + 25, "half-moon arc + bench pin\n(sweeps tray + leather skin below)\nYOUR SEAT", ha="center", va="top", fontsize=7.5, color=INK)
rect(ax, 520, -170, 360, 580, MUTED, None, alpha=0.08, r=4)
ax.text(700, -150, "drawer unit under (6 drawers)", ha="center", va="top", fontsize=7, color=MUTED)
rect(ax, C["x0"], C["y0"], C["x1"] - C["x0"], C["y1"] - C["y0"], HOT, None)
ax.text((C["x0"] + C["x1"]) / 2, C["y1"] + 15, "steel hot-work cart 600 x 450 (rolls away)", ha="center", va="bottom", fontsize=7.5, color=HOT)
rect(ax, C["x0"] + 35, -60, 260, 260, HOT, "kiln", alpha=0.25)
rect(ax, C["x1"] - 280, -50, 260, 240, HOT, "fixed torch\n+ hood", alpha=0.25)
ax.add_patch(mp.Circle((-1700, WALL + 70), 50, fc=HOT, alpha=0.6, ec=HOT))
ax.text(-1700, WALL + 130, "extinguisher\n(away from\nthe flame)", ha="center", va="bottom", fontsize=6.5, color=INK)
ax.set_xlim(-1800, 1300)
ax.set_ylim(-300, 720)
ax.invert_yaxis()
ax.set_aspect("equal")
ax.set_title("Top view: you stand at the bottom edge", color=INK, fontsize=11, loc="left")
ax.text(-445, 290, "arm work area at table height", fontsize=6.5, color=ROBOT, va="bottom")

# ---------------------------------------------------------------- wall elevation
ax2.add_patch(mp.Rectangle((-1800, -760), 3100, 2300, fc="#fbfaf7", ec="none"))
ax2.add_patch(mp.Rectangle((PEG["x0"], PEG["z0"]), PEG["x1"] - PEG["x0"], PEG["z1"] - PEG["z0"], fc="#e7d5bd", ec="#b89c7a", lw=1))
for x in range(int(PEG["x0"]) + 25, int(PEG["x1"]), 50):
    for z in range(int(PEG["z0"]) + 25, int(PEG["z1"]), 50):
        ax2.plot(x, z, ".", ms=0.8, color="#8a7458")
zones = [
    (-880, 860, 170, 260, "hammer,\nscrew-\ndrivers"),
    (-605, 870, 125, 220, "pliers\nx3"),
    (-470, 820, 410, 290, "saws + files"),
    (-30, 840, 400, 260, "measuring:\nrule, calipers,\nsquare, tape"),
    (-170, 700, 580, 95, "parts bins x6"),
    (-150, 480, 600, 210, "shelf: glue, oil,\npolish + filament"),
    (500, 800, 260, 310, "F-clamps,\ncordless\ndrill"),
    (500, 600, 280, 120, "solder / wire"),
    (775, 880, 95, 190, "PPE"),
    (-880, 500, 260, 120, "paper towel"),
]
for x0, z0, w, h, t in zones:
    rect(ax2, x0, z0, w, h, HUMAN, t, alpha=0.22, fs=7)
ax2.add_patch(mp.Rectangle((-880, 1230), 1760, 15, fc="#eda100", ec="none"))
ax2.text(0, 1255, "LED light bar", ha="center", va="bottom", fontsize=7, color=MUTED)
ax2.add_patch(mp.Rectangle((B["x0"], -40), B["x1"] - B["x0"], 40, fc="#c9a77f", ec="none"))
ax2.add_patch(mp.Rectangle((-500, 0), 1000, 25, fc=INK))
rect(ax2, 511, 0, 389, 458, PRINT, "P1S", alpha=0.25)
ax2.add_patch(mp.Rectangle((C["x0"] - 60, C["top"] - 20), C["x1"] - C["x0"] + 100, 920 - C["top"], fc="#c8ccd0", ec="#9aa0a6"))
ax2.text((C["x0"] + C["x1"]) / 2, 880, "steel heat panel\n(no pegboard\nnear the torch)", ha="center", va="top", fontsize=7.5, color=INK)
rect(ax2, C["x0"], C["top"] - 30, C["x1"] - C["x0"], 30, HOT, None, alpha=0.6)
rect(ax2, C["x0"] + 35, C["top"], 260, 320, HOT, "kiln", alpha=0.3)
rect(ax2, C["x1"] - 280, C["top"], 260, 260, HOT, "torch", alpha=0.3)
rect(ax2, C["x1"] - 300, C["top"] + 450, 300, 20, HOT, None, alpha=0.6)
ax2.text(C["x1"] - 150, C["top"] + 480, "fume hood", ha="center", va="bottom", fontsize=7, color=INK)
ax2.add_patch(mp.FancyBboxPatch((-1750, 150), 100, 370, boxstyle="round,pad=0,rounding_size=30", fc=HOT, alpha=0.7, ec=HOT))
ax2.set_xlim(-1800, 1300)
ax2.set_ylim(-100, 1350)
ax2.set_aspect("equal")
ax2.set_title("Wall elevation (z above the bench top, mm)", color=INK, fontsize=11, loc="left")

for a in (ax, ax2):
    a.tick_params(colors=MUTED, labelsize=7)
    for s in a.spines.values():
        s.set_visible(False)
handles = [mp.Patch(fc=c, alpha=0.4, ec=c, label=t) for c, t in ((ROBOT, "robot stations (IK reach checked)"), (HUMAN, "your hand work + wall storage"), (HOT, "hot work (you only)"), (PRINT, "printing"))]
fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, fontsize=8.5)
fig.suptitle("Wall workshop around the Gantry Arm", x=0.02, ha="left", color=INK, fontsize=14)
fig.tight_layout(rect=(0, 0.03, 1, 0.97))
out = HERE / "render" / "out" / "workshop_plan.png"
out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out, facecolor="white")
print("wrote", out)
