"""Arm v2 renders (steppers + planetary on linear rails).

    python v2/render_v2.py --view hero|drive|wrist|gantry [--samples 96] [--res 2400x1500]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(ROOT / "render"), str(ROOT / "sim"), str(ROOT)]

import bpy  # noqa: E402
from mathutils import Quaternion, Vector  # noqa: E402

import detail_materials as DM  # noqa: E402
import fk  # noqa: E402
import render_blender as RB  # noqa: E402
from render_mechanics import obj  # noqa: E402

OUT = HERE / "out"
TOOL_PARTS = ("coupler_robot", "coupler_magnets", "coupler_pogo", "coupler_lock", "gripper_", "j5_roll_flange")
POSES = {
    "hero": {"gantry_x": 420.0, "gantry_z": 160.0, "base_yaw": 20.0, "shoulder_lift": 38.0, "elbow_flex": 62.0, "wrist_flex": 45.0, "wrist_roll": 0.0},
    "drive": {"gantry_x": 420.0, "gantry_z": 160.0, "base_yaw": 0.0, "shoulder_lift": 15.0, "elbow_flex": 30.0, "wrist_flex": 20.0, "wrist_roll": 0.0},
}


def materials():
    mats = RB.make_materials("white")
    P = RB.principled
    for k, m in list(mats.items()):
        if k not in ("accent", "lens", "estop_red", "estop_yellow", "floor"):
            DM.add_bevel(m, 0.0005)
    mats.update({
        "shell": DM.printed("v2_shell", (0.82, 0.82, 0.80), 0.48, 0.16, 0.10),
        "cap": DM.printed("v2_cap", (0.20, 0.21, 0.22), 0.5, 0.16, 0.10),
        "metal": DM.brushed("v2_metal"),
        "motor": P("v2_motor", (0.03, 0.03, 0.033), 0.35, metal=0.6),
        "motor_cap": DM.brushed("v2_mcap", (0.55, 0.56, 0.58), 0.3),
        "gearbox": DM.brushed("v2_gb", (0.34, 0.35, 0.37), 0.3),
        "block": DM.brushed("v2_block", (0.62, 0.63, 0.65), 0.25),
        "extrusion": P("v2_anod", (0.05, 0.05, 0.055), 0.32, metal=0.85),
        "rubber": DM.rubber("v2_belt", (0.03, 0.03, 0.03)),
        "steel_gear": DM.brushed("v2_gear", (0.62, 0.63, 0.66), 0.28),
    })
    return mats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", default="hero", choices=["hero", "drive", "wrist", "gantry"])
    ap.add_argument("--samples", type=int, default=96)
    ap.add_argument("--res", default="2400x1500")
    ap.add_argument("--suffix", default="")
    a = ap.parse_args()
    pose = POSES["drive" if a.view in ("drive", "wrist") else "hero"]
    poses = fk.body_poses(pose)
    sc = RB.reset()
    mats = materials()
    section = a.view in ("drive", "wrist")
    cut_names = {m["name"] for m in json.loads((OUT / "cut" / "manifest.json").read_text())} if section else set()
    for m in json.loads((OUT / "manifest.json").read_text()):
        if m["body"] not in poses or m["name"] in cut_names:
            continue
        obj(OUT / "stl" / f"{m['name']}.stl", poses[m["body"]], mats.get(m["role"], mats["shell"]))
    if section:
        for m in json.loads((OUT / "cut" / "manifest.json").read_text()):
            if m["part"] == "keep" and m["body"] in poses:
                obj(OUT / "cut" / "keep" / f"{m['name']}.stl", poses[m["body"]], mats.get(m["role"], mats["shell"]))
    for m in json.loads((ROOT / "cad" / "out" / "manifest.json").read_text()):  # coupler + gripper from the tool system
        if m["body"] == "tool" and m["name"].startswith(TOOL_PARTS):
            obj(ROOT / "cad" / "out" / "stl" / f"{m['name']}.stl", poses["tool"], mats.get(m["role"], mats["shell"]))
    DM.sky_world(0.035)
    sc.view_settings.exposure = -0.5

    def w(body, local=(0, 0, 0)):
        pos, quat = poses[body]
        return (Vector(pos) + Quaternion(quat) @ Vector(local)) / 1000

    if a.view == "hero":
        tgt = (w("turret", (0, 0, 60)) + w("wrist")) / 2 + Vector((0, 0, -0.08))
        loc = tgt + Vector((0.62, 0.78, 0.28))
        lens, fst = 40, None
    elif a.view == "drive":
        tgt = (w("turret", (0, 0, 120)) + w("forearm")) / 2
        d = Quaternion(poses["turret"][1]) @ Vector((1, 0, 0))
        loc = tgt + d * 1.0 + Vector((0.30, 0.0, 0.20))
        lens, fst = 45, None
    elif a.view == "wrist":
        tgt = w("wrist", (0, 0, 10))
        d = Quaternion(poses["wrist"][1]) @ Vector((1, 0, 0))
        loc = tgt + d * 0.42 + Vector((0.10, 0.0, 0.16))
        lens, fst = 55, 8.0
    else:  # gantry: rails and carriages
        tgt = w("z_carriage", (0, 0, -80))
        loc = tgt + Vector((0.55, 0.75, 0.10))
        lens, fst = 40, None
    RB.camera(loc, tgt, lens=lens, fstop=fst)
    RB.studio(mats, tgt - loc)
    sc.cycles.samples = a.samples
    sc.render.resolution_x, sc.render.resolution_y = (int(v) for v in a.res.split("x"))
    out = HERE / "out" / f"v2_{a.view}{a.suffix}.png"
    sc.render.filepath = str(out)
    bpy.ops.render.render(write_still=True)
    print("wrote", out)


if __name__ == "__main__":
    main()
