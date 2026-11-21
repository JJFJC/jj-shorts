"""Render: Gantry Arm with the compliant sanding spindle on a small part held in the servo vise;
the fine-sand and polish spindles wait in the tool rack.

    python render/render_finishing.py [--view close|wide] [--samples 96]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "render"), str(ROOT / "sim"), str(ROOT), str(ROOT / "software" / "lerobot_robot_gantry_arm")]

import bpy  # noqa: E402
from mathutils import Quaternion, Vector  # noqa: E402

import fk  # noqa: E402
import render_blender as RB  # noqa: E402
from lerobot_robot_gantry_arm.kinematics import Geometry, Kinematics, Pose  # noqa: E402

FIN = ROOT / "cad" / "out" / "finishing"
SKIP = ("gripper_", "tool_", "coupler_contact_pads", "coupler_tool_half")


def obj(path, pose, material):
    bpy.ops.wm.stl_import(filepath=str(path))
    o = bpy.context.selected_objects[0]
    pos, quat = pose
    o.scale = (0.001, 0.001, 0.001)
    o.rotation_mode = "QUATERNION"
    o.rotation_quaternion = Quaternion(quat)
    o.location = Vector(pos) / 1000
    o.data.materials.append(material)
    bpy.context.view_layer.objects.active = o
    try:
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", default="close", choices=["close", "wide", "macro"])
    ap.add_argument("--finish", default="white", choices=["white", "graphite"])
    ap.add_argument("--samples", type=int, default=96)
    ap.add_argument("--res", default="1700x1050")
    ap.add_argument("--suffix", default="")
    ap.add_argument("--detail", action="store_true", help="procedural materials, bevel highlights, wood bench, sky light")
    a = ap.parse_args()
    man = json.loads((FIN / "manifest.json").read_text())
    tcp = next(m["tcp_mm"] for m in man if "tcp_mm" in m and m["tool"] == "sand")
    g = Geometry.load()
    # tool frame origin = J5 flange; the default gripper TCP sits 80 mm out, this tool at `tcp`
    k = Kinematics(Geometry(**{**g.__dict__, "tool": g.tool - 80.0 + tcp}))
    # sanding the top edge of the pendant (vise at y=205, jaw top ~z 58, part top ~z 62)
    q = k.ik(Pose(6.0, 203.0, 70.0, 180.0, 90.0))
    k_dock = Kinematics()  # rack docking uses the standard TCP convention
    rack = json.loads((ROOT / "software" / "lerobot_robot_gantry_arm" / "lerobot_robot_gantry_arm" / "assets" / "geometry.json").read_text())["rack"]
    sc = RB.reset()
    mats = RB.make_materials(a.finish)
    mats.update({
        "abrasive": RB.principled("abr", (0.32, 0.20, 0.10), 0.95),
        "abrasive_fine": RB.principled("abrf", (0.55, 0.45, 0.32), 0.9),
        "felt": RB.principled("felt", (0.85, 0.85, 0.82), 1.0),
        "part": RB.principled("part", (0.70, 0.12, 0.25), 0.35, coat=0.6),
        "motor": RB.principled("motor2", (0.04, 0.04, 0.045), 0.35, metal=0.6),
    })
    if a.detail:
        import detail_materials as DM

        white = a.finish == "white"
        shell_c = (0.82, 0.82, 0.80) if white else (0.035, 0.037, 0.04)
        cap_c = (0.20, 0.21, 0.22) if white else (0.09, 0.093, 0.10)
        for k_, m_ in list(mats.items()):
            if k_ not in ("accent", "lens", "estop_red", "estop_yellow", "floor"):
                DM.add_bevel(m_, 0.0006)
        mats.update({
            "shell": DM.printed("d_shell", shell_c, 0.5, 0.16, 0.12),
            "cap": DM.printed("d_cap", cap_c, 0.5, 0.16, 0.12),
            "abrasive": DM.sandpaper("d_abr", (0.42, 0.26, 0.13), 4.0),
            "abrasive_fine": DM.sandpaper("d_abrf", (0.62, 0.52, 0.38), 7.0),
            "felt": DM.rubber("d_felt", (0.86, 0.86, 0.83)),
            "metal": DM.brushed("d_metal"),
            "steel": DM.brushed("d_steel"),
            "screw": DM.brushed("d_screw", (0.06, 0.06, 0.065), 0.35),
            "motor": DM.brushed("d_motor", (0.05, 0.05, 0.055), 0.3),
            "rubber": DM.rubber("d_rubber"),
            "part": DM.printed("d_part", (0.72, 0.12, 0.28), 0.38, 0.20, 0.45),
            "floor": DM.wood(),
        })
        DM.sky_world(0.035)
        sc.view_settings.exposure = -0.4
    mats.setdefault("screw", RB.principled("screw", (0.06, 0.06, 0.065), 0.35, metal=1.0))
    # robot
    poses = fk.body_poses(q)
    for m in json.loads((ROOT / "cad" / "out" / "manifest.json").read_text()):
        if m["body"] in poses and not m["name"].startswith(SKIP):
            obj(ROOT / "cad" / "out" / "stl" / f"{m['name']}.stl", poses[m["body"]], mats.get(m["role"], mats["shell"]))
        elif m["body"] == "world" and m["name"].startswith("tool_rack"):
            obj(ROOT / "cad" / "out" / "stl" / f"{m['name']}.stl", ([0, 0, 0], [1, 0, 0, 0]), mats.get(m["role"], mats["shell"]))
    # mounted sanding spindle
    for m in man:
        if m.get("tool") == "sand":
            obj(FIN / f"{m['name']}.stl", poses["tool"], mats.get(m["role"], mats["shell"]))
    # fine + polish spindles parked in rack slots 1 and 2 (docked tool-frame pose from the IK + FK)
    xs = {int(s): v for s, v in rack["slots"].items()}
    qd = k_dock.ik(Pose(xs[1], rack["y"], rack["tcp_z"], 180.0, rack["roll_deg"]))
    dpos, dquat = fk.body_poses(qd)["tool"]
    for slot, key in ((1, "fine"), (2, "polish")):
        pos = [dpos[0] + xs[slot] - xs[1], dpos[1], dpos[2] + (tcp - 80.0) * 0]
        for m in man:
            if m.get("tool") == key:
                obj(FIN / f"{m['name']}.stl", (pos, dquat), mats.get(m["role"], mats["shell"]))
    for m in man:
        if m["body"] == "world":
            obj(FIN / f"{m['name']}.stl", ([0, 0, 0], [1, 0, 0, 0]), mats.get(m["role"], mats["shell"]))
    if a.view == "close":
        tgt = Vector((0.0, 0.20, 0.10))
        loc = tgt + Vector((0.34, -0.40, 0.20))
        RB.camera(loc, tgt, lens=55, fstop=6.3)
    elif a.view == "macro":
        tgt = Vector((0.004, 0.203, 0.078))
        loc = tgt + Vector((0.16, -0.17, 0.07))
        RB.camera(loc, tgt, lens=70, fstop=4.0)
    else:
        tgt = Vector((0.16, 0.22, 0.22))
        loc = tgt + Vector((0.55, -1.10, 0.62))
        RB.camera(loc, tgt, lens=36)
    RB.studio(mats, tgt - loc)
    sc.cycles.samples = a.samples
    sc.render.resolution_x, sc.render.resolution_y = (int(v) for v in a.res.split("x"))
    out = ROOT / "render" / "out" / f"finishing_{a.view}{a.suffix}.png"
    sc.render.filepath = str(out)
    bpy.ops.render.render(write_still=True)
    print("pose", {k: round(v, 1) for k, v in q.items()})
    print("wrote", out)


if __name__ == "__main__":
    main()
