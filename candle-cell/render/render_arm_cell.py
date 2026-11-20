"""Render the Gantry Arm candle cell: the real Gantry Arm CAD, posed by its own IK, holding a
candle on the sanding drum, with every station around it.

    python render/render_arm_cell.py [--view overview|sanding|printer] [--samples 96]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GA = ROOT.parent / "gantry-arm"
sys.path[:0] = [str(GA / "render"), str(GA / "sim"), str(GA), str(GA / "software" / "lerobot_robot_gantry_arm")]

import bpy  # noqa: E402
from mathutils import Quaternion, Vector  # noqa: E402

import fk  # noqa: E402  (gantry-arm/sim)
import render_blender as RB  # noqa: E402  (gantry-arm/render)
from lerobot_robot_gantry_arm.kinematics import Geometry, Kinematics, Pose  # noqa: E402

CELL = ROOT / "cad" / "out" / "arm_cell"
SKIP = ("gripper_", "tool_", "coupler_contact_pads")  # default gripper + spare tools are swapped for the candle gripper


def pose_for(view: str) -> dict:
    lay = json.loads((CELL / "layout.json").read_text())
    STATIONS, TOOL_TCP = lay["stations"], lay["tool_tcp"]

    g = Geometry.load()
    k = Kinematics(Geometry(**{**g.__dict__, "tool": g.tool - 64.0 + TOOL_TCP}))  # candle gripper is longer
    if view == "printer":
        x, y, z = STATIONS["flex_lip"]
        target = Pose(x - 70, y, z + 30, 135.0, 90.0)
    else:
        x, y, z = STATIONS["sander"]
        target = Pose(x, y, z + 31.0, 90.0, 0.0)  # candle (r 30) resting on the drum top
        # J1 straight behind the sander so the candle axis lines up with the drum axis (yaw 0)
        return k.ik(target, gantry_x=x - k.g.x0 - k.g.lateral)
    q = k.ik(target)
    return q


def load_arm(mats, q):
    manifest = json.loads((GA / "cad" / "out" / "manifest.json").read_text())
    poses = fk.body_poses(q)
    for m in manifest:
        if m["body"] not in poses or m["name"].startswith(SKIP):
            continue
        RB_obj(GA / "cad" / "out" / "stl" / f"{m['name']}.stl", poses[m["body"]], mats.get(m["role"], mats["shell"]))
    return poses


def RB_obj(path, pose, material):
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
    ap.add_argument("--view", default="overview", choices=["overview", "sanding", "printer"])
    ap.add_argument("--finish", default="white", choices=["white", "graphite"])
    ap.add_argument("--samples", type=int, default=96)
    ap.add_argument("--res", default="1700x1050")
    ap.add_argument("--suffix", default="")
    a = ap.parse_args()
    q = pose_for("printer" if a.view == "printer" else "sanding")
    print("pose:", {k: round(v, 1) for k, v in q.items()})
    sc = RB.reset()
    mats = RB.make_materials(a.finish)
    mats.update({
        "printer": RB.principled("printer", (0.10, 0.10, 0.11), 0.4),
        "frame": RB.principled("frame2", (0.05, 0.055, 0.06), 0.5),
        "steel": RB.principled("steel2", (0.72, 0.73, 0.75), 0.25, metal=1.0),
        "dark": RB.principled("dark2", (0.015, 0.015, 0.018), 0.4),
        "mold": RB.principled("mold", (0.05, 0.42, 0.48), 0.35),
        "mold2": RB.principled("mold2", (0.03, 0.22, 0.26), 0.4),
        "wax": RB.principled("wax", (0.93, 0.89, 0.80), 0.55),
        "wax_liquid": RB.principled("waxl", (0.95, 0.78, 0.45), 0.1, transmission=0.3),
        "abrasive": RB.principled("abrasive", (0.30, 0.18, 0.10), 0.95),
    })
    poses = load_arm(mats, q)
    for m in json.loads((CELL / "manifest.json").read_text()):
        pose = poses[m["body"]] if m["body"] == "tool" else ([0, 0, 0], [1, 0, 0, 0])
        RB_obj(CELL / f"{m['name']}.stl", pose, mats.get(m["role"], mats["shell"]))
    if a.view == "overview":
        tgt = Vector((-0.12, 0.22, 0.25))
        loc = tgt + Vector((0.55, -1.95, 1.05))
        RB.camera(loc, tgt, lens=30)
    elif a.view == "sanding":
        tcp = Vector(poses["tool"][0]) / 1000
        tgt = tcp + Vector((0.0, 0.03, -0.04))
        loc = tgt + Vector((0.42, -0.35, 0.22))
        RB.camera(loc, tgt, lens=50, fstop=5.6)
    else:
        tgt = Vector((-0.60, 0.28, 0.16))
        loc = tgt + Vector((0.45, 0.95, 0.55))
        RB.camera(loc, tgt, lens=40)
    RB.studio(mats, tgt - loc)
    sc.cycles.samples = a.samples
    sc.render.resolution_x, sc.render.resolution_y = (int(v) for v in a.res.split("x"))
    out = ROOT / "render" / "out" / f"arm_cell_{a.view}{a.suffix}.png"
    sc.render.filepath = str(out)
    bpy.ops.render.render(write_still=True)
    print("wrote", out)


if __name__ == "__main__":
    main()
