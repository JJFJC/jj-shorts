"""Run the skills in the MuJoCo sim and plot what the arm actually did -> docs/img/demo_*.png

    python software/demos/sim_demos.py
"""

from __future__ import annotations

import logging
import os
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "lerobot_robot_gantry_arm"))
logging.disable(logging.WARNING)

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import cv2  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from lerobot_robot_gantry_arm import GantryArm, GantryArmConfig  # noqa: E402
from lerobot_robot_gantry_arm.kinematics import WORK_AREA, Pose  # noqa: E402
from lerobot_robot_gantry_arm.skills import pick_place, plotter, tool_changer, vision  # noqa: E402
from lerobot_robot_gantry_arm.skills.motion import Motion  # noqa: E402

OUT = HERE.parents[1] / "docs" / "img"
INK, INK2, MUTED, GRID = "#1f1f1e", "#5c5b55", "#8f8e86", "#e4e3dd"
S1, S2, S3, S4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"  # categorical slots 1-4

plt.rcParams.update({
    "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False, "figure.facecolor": "white",
    "legend.frameon": False, "legend.fontsize": 9,
})

FACE_SVG = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 300 160">
<rect x="10" y="10" width="280" height="140" rx="30" fill="none" stroke="black"/>
<circle cx="105" cy="65" r="26" fill="none" stroke="black"/><circle cx="105" cy="65" r="9" fill="none" stroke="black"/>
<circle cx="195" cy="65" r="26" fill="none" stroke="black"/><circle cx="195" cy="65" r="9" fill="none" stroke="black"/>
<path d="M100 115 Q150 140 200 115" fill="none" stroke="black"/>
<path d="M150 10 L150 -2 M140 -2 L160 -2" fill="none" stroke="black"/></svg>"""


def robot():
    r = GantryArm(GantryArmConfig(backend="sim", id="demo", calibration_dir=Path(tempfile.mkdtemp()), max_relative_target=None, payload_g=150))
    r.connect()
    m = Motion(r)
    m.move_to(Pose(0, 200, 120, 180))
    m.trace.clear()
    return r, m


def work_area(ax):
    (x0, x1), (y0, y1) = WORK_AREA["x"], WORK_AREA["y"]
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=MUTED, ls=(0, (4, 3)), lw=1, label="guaranteed work area"))


def demo_plotter():
    r, m = robot()
    r.sim.tool_id = plotter.PEN_TOOL_ID
    p = plotter.PlotParams(speed_mm_s=60)
    strokes = plotter.plan(FACE_SVG)
    plotter.plot(m, strokes, p)
    fig, ax = plt.subplots(figsize=(8, 4.6))
    for i, s in enumerate(strokes):
        a = np.array(s)
        ax.plot(a[:, 0], a[:, 1], color=GRID, lw=6, solid_capstyle="round", label="planned (from SVG)" if i == 0 else None, zorder=1)
    down = np.array([tr[3] for tr in m.trace if tr[1][2] < p.pen_down_z + 0.3])
    ax.scatter(down[:, 0], down[:, 1], s=2, color=S1, label="pen tip in sim (pen down)", zorder=2)
    err = [np.min(np.linalg.norm(np.array([pt for s in strokes for pt in s]) - d[:2], axis=1)) for d in down[::5]]
    work_area(ax)
    ax.set_aspect("equal")
    ax.set_xlim(-200, 200)
    ax.set_ylim(100, 310)
    ax.set_xlabel("table x (mm, along the rail)")
    ax.set_ylabel("table y (mm)")
    ax.set_title(f"Pen plotter: SVG drawn by the simulated arm\n{len(strokes)} strokes, pen tip within {np.percentile(err, 95):.1f} mm of the drawing (95th pct)", loc="left")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=3)
    fig.tight_layout()
    fig.savefig(OUT / "demo_plotter.png", dpi=150)
    plt.close(fig)
    r.disconnect()


def demo_sorting():
    objs = {"red_1": (-300.0, 170.0), "blue_1": (-60.0, 240.0), "yellow_1": (120.0, 150.0)}
    bins = {"red": (330.0, 160.0), "blue": (330.0, 220.0), "yellow": (330.0, 280.0)}
    img, _ = vision.synthetic_table_image(objs)
    cal = vision.TableCalibration.from_image(img)
    dets = vision.find_colour_objects(img, cal)
    r, m = robot()
    pick_place.sort_by_colour(m, dets, bins, object_h_mm=25.0)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.8), gridspec_kw={"width_ratios": [1, 1.3]})
    vis = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).copy()
    for tid, c in vision.detect_tags(img).items():
        cv2.polylines(vis, [c.astype(int)], True, (42, 120, 214), 2)
    a1.imshow(vis)
    for d in dets:
        a1.annotate(f"{d.label}\n({d.xy_mm[0]:.0f}, {d.xy_mm[1]:.0f}) mm", d.px, xytext=(10, -28), textcoords="offset points", fontsize=8, color=INK,
                    arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    a1.set_title(f"Eyes camera: 4 AprilTags → table calibration ({cal.rms_mm:.1f} mm RMS)", loc="left")
    a1.axis("off")
    path = np.array([tr[3] for tr in m.trace])
    holding = np.array([tr[2] < 50 for tr in m.trace])
    a2.plot(path[:, 0], path[:, 1], color=MUTED, lw=1, label="tool path (empty)")
    a2.scatter(path[holding, 0], path[holding, 1], s=3, color=S1, label="carrying an object", zorder=3)
    marks = {"red": S2, "blue": S1, "yellow": S4}
    for d in dets:
        a2.scatter(*d.xy_mm, s=90, marker="o", facecolor="white", edgecolor=INK2, lw=1.2, zorder=4)
        a2.annotate(f"{d.label} start", d.xy_mm, xytext=(-10, -16), textcoords="offset points", fontsize=8, color=INK, ha="right")
    for name, xy in bins.items():
        a2.scatter(*xy, s=110, marker="s", facecolor="white", edgecolor=INK, lw=1.2, zorder=4)
        a2.annotate(f"{name} bin", xy, xytext=(8, -3), textcoords="offset points", fontsize=8, color=INK)
    work_area(a2)
    a2.set_aspect("equal")
    a2.set_xlim(-470, 470)
    a2.set_ylim(95, 320)
    a2.set_xlabel("table x (mm)")
    a2.set_ylabel("table y (mm)")
    a2.set_title("Colour sorting: pick → carry → place, simulated", loc="left")
    a2.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=3)
    fig.tight_layout()
    fig.savefig(OUT / "demo_sorting.png", dpi=150)
    plt.close(fig)
    r.disconnect()


def demo_tool_change():
    r, m = robot()
    rack = tool_changer.Rack.load()
    tool_changer.change_to(m, 3, rack)
    t = np.array([tr[0] for tr in m.trace])
    tcp = np.array([tr[3] for tr in m.trace])
    gz = np.array([tr[4]["gantry_z"] for tr in m.trace])
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True, gridspec_kw={"height_ratios": [1.4, 1]})
    a1.plot(t, tcp[:, 2], color=S1, lw=2)
    a1.set_ylabel("tool height z (mm)")
    a1.set_title("Automatic tool change (gripper → pen), simulated: the gantry Z, not the servos, breaks the magnets", loc="left")
    a2.plot(t, gz, color=S2, lw=2)
    a2.set_ylabel("gantry Z (mm)")
    a2.set_xlabel("time (s)")
    # annotate phases from the motion
    stops = [i for i in range(1, len(t)) if abs(gz[i] - gz[i - 1]) > 1e-6 and abs(m.trace[i][4]["shoulder_lift"] - m.trace[i - 1][4]["shoulder_lift"]) < 1e-9]
    if stops:
        groups = np.split(np.array(stops), np.where(np.diff(stops) > 2)[0] + 1)
        labels = ["lift: fork holds gripper,\nmagnets release", "lower onto pen:\ncone aligns, magnets snap"]
        for g, lab in zip(groups, labels):
            mid = t[g[len(g) // 2]]
            a2.annotate(lab, (mid, gz[g[len(g) // 2]]), xytext=(18, 10), textcoords="offset points", fontsize=8, color=INK,
                        arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
            for ax in (a1, a2):
                ax.axvspan(t[g[0]], t[g[-1]], color=GRID, alpha=0.6, lw=0)
    a1.annotate(f"docked in slot (z = {rack.tcp_z:.0f} mm)", (t[np.argmin(tcp[:, 2])], tcp[:, 2].min()), xytext=(15, 25), textcoords="offset points",
                fontsize=8, color=INK, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    fig.tight_layout()
    fig.savefig(OUT / "demo_tool_change.png", dpi=150)
    plt.close(fig)
    r.disconnect()


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    demo_plotter()
    demo_sorting()
    demo_tool_change()
    print("wrote", sorted(p.name for p in OUT.glob("demo_*.png")))
