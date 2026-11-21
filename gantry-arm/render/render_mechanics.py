"""Inside-the-arm renders: cut-away (shells sectioned on the arm plane) and an exploded shoulder.

    python render/render_mechanics.py --view cutaway|shoulder|wrist|exploded [--samples 96] [--res 2400x1500]

Inputs: cad/out (shells, gantry), cad/out/cut_x and cut_y (sectioned + ghost halves), cad/out/internals (python cad/internals.py).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "render"), str(ROOT / "sim"), str(ROOT)]

import bpy  # noqa: E402
from mathutils import Quaternion, Vector  # noqa: E402

import detail_materials as DM  # noqa: E402
import fk  # noqa: E402
import render_blender as RB  # noqa: E402

OUT = ROOT / "cad" / "out"
POSE = {"gantry_x": 0.0, "gantry_z": 120.0, "base_yaw": 0.0, "shoulder_lift": 35.0, "elbow_flex": 65.0, "wrist_flex": 40.0, "wrist_roll": 0.0}
SKIP_TOOLS = ("tool_", "coupler_tool_half", "coupler_contact_pads")
# exploded shoulder: offset along the J2 axis (turret +Y) per part-name prefix, mm
EXPLODE = [
    ("j2_cap_outer", 150.0), ("j2_shaft", 110.0), ("j2_bearing_2", 95.0), ("j2_bearing_1", 80.0), ("j2_bearing_boss", 65.0),
    ("j2_hub_gear", 45.0), ("j2_pinion", 28.0), ("j2_horn", 16.0), ("j2_servo", 0.0), ("j2_mount_plate", -30.0), ("j2_cap_inner", -60.0),
    ("j2_cable_loop", 6.0), ("j2_shoulder_puck", -0.0),
]


def obj(path, pose, material, extra_offset=(0.0, 0.0, 0.0)):
    bpy.ops.wm.stl_import(filepath=str(path))
    o = bpy.context.selected_objects[0]
    pos, quat = pose
    q = Quaternion(quat)
    off = q @ Vector(extra_offset)
    o.scale = (0.001, 0.001, 0.001)
    o.rotation_mode = "QUATERNION"
    o.rotation_quaternion = q
    o.location = (Vector(pos) + off) / 1000
    o.data.materials.append(material)
    bpy.context.view_layer.objects.active = o
    try:
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(30))
    except Exception:
        pass
    return o


def materials():
    mats = RB.make_materials("white")
    P = RB.principled
    for k_, m_ in list(mats.items()):
        if k_ not in ("accent", "lens", "estop_red", "estop_yellow", "floor"):
            DM.add_bevel(m_, 0.0005)
    mats.update({
        "shell": DM.printed("m_shell", (0.82, 0.82, 0.80), 0.5, 0.16, 0.10),
        "cap": DM.printed("m_cap", (0.20, 0.21, 0.22), 0.5, 0.16, 0.10),
        "metal": DM.brushed("m_metal"),
        "steel_gear": DM.brushed("m_gear", (0.62, 0.63, 0.66), 0.28),
        "servo": P("m_servo", (0.025, 0.027, 0.03), 0.45, spec=0.4),
        "rubber": DM.rubber("m_seal", (0.12, 0.03, 0.02)),  # red-brown bearing seals
        "cable": P("m_cable", (0.85, 0.32, 0.05), 0.55),  # orange harness sleeve
        "pcb": P("m_pcb", (0.05, 0.30, 0.12), 0.35),
        "accent_white": P("m_pom", (0.92, 0.92, 0.90), 0.25, coat=0.3),
        "screw": DM.brushed("m_screw", (0.06, 0.06, 0.065), 0.35),
    })
    return mats


def ghost_material():
    m = RB.principled("m_ghost", (0.85, 0.88, 0.92), 0.25)
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Alpha"].default_value = 0.04
    return m


def load_scene(view, mats, show_ghost=False):
    poses = fk.body_poses(POSE)
    ghost = ghost_material()
    cut = "y" if view in ("overview", "gears") else "x"
    cdir = OUT / f"cut_{cut}"
    cman = json.loads((cdir / "manifest.json").read_text())
    cut_names = {m["name"] for m in cman}
    exploded = view == "exploded"

    def off_for(name):
        return next((d for p, d in EXPLODE if name.startswith(p)), 0.0)

    def place(path, m, mat):
        if exploded:
            if m["body"] == "upper_arm":  # rotor parts: show on the turret's J2 axis
                return obj(path, poses["turret"], mat, (0.0, off_for(m["name"]), 120.0))
            return obj(path, poses[m["body"]], mat, (0.0, off_for(m["name"]), 0.0))
        return obj(path, poses[m["body"]], mat)

    for m in json.loads((OUT / "manifest.json").read_text()):  # gantry + arm parts that have no cut version
        if m["name"].startswith(SKIP_TOOLS) or m["body"] not in poses or m["name"] in cut_names:
            continue
        if exploded and not m["name"].startswith("j2_"):
            continue
        place(OUT / "stl" / f"{m['name']}.stl", m, mats.get(m["role"], mats["shell"]))
    for m in cman:  # sectioned shells (+ internals in the y cut): kept half solid, removed half as a ghost
        if m["body"] not in poses or (exploded and not m["name"].startswith("j2_")):
            continue
        if m["part"] == "ghost" and not show_ghost:
            continue
        mat = ghost if m["part"] == "ghost" else mats.get(m["role"], mats["shell"])
        place(cdir / m["part"] / f"{m['name']}.stl", m, mat)
    if cut == "x":  # internals whole
        for m in json.loads((OUT / "internals" / "manifest.json").read_text()):
            if m["body"] not in poses or (exploded and not m["name"].startswith("j2_")):
                continue
            place(OUT / "internals" / f"{m['name']}.stl", m, mats.get(m["role"], mats["metal"]))
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", default="overview", choices=["overview", "gears", "shoulder", "wrist", "exploded"])
    ap.add_argument("--samples", type=int, default=96)
    ap.add_argument("--res", default="2400x1500")
    ap.add_argument("--suffix", default="")
    ap.add_argument("--ghost", action="store_true", help="show the removed halves as faint glass")
    a = ap.parse_args()
    sc = RB.reset()
    mats = materials()
    poses = load_scene(a.view, mats, a.ghost)
    DM.sky_world(0.035)
    sc.view_settings.exposure = -0.5

    def w(body, local=(0, 0, 0)):  # a point in a body frame -> world metres
        pos, quat = poses[body]
        return (Vector(pos) + Quaternion(quat) @ Vector(local)) / 1000

    # the section plane is body x = 0; body +x points to world +y at base_yaw 0, so look from +y
    # x-section views look along the section normal (turret/wrist body +x); y-section views look along body +y
    if a.view == "overview":
        tgt = (w("turret", (0, 0, 120)) + w("wrist")) / 2
        d = Quaternion(poses["turret"][1]) @ Vector((0, 1, 0))
        loc = tgt + d * 0.95 + Vector((0.25, 0.0, 0.22))
        lens, fst = 38, None
    elif a.view == "gears":
        tgt = (w("turret", (0, 25, 120)) + w("forearm", (0, 25, 0))) / 2
        d = Quaternion(poses["turret"][1]) @ Vector((0, 1, 0))
        loc = tgt + d * 0.55 + Vector((0.12, 0.0, 0.10))
        lens, fst = 50, 8.0
    elif a.view == "shoulder":
        tgt = w("turret", (0, 0, 95))
        d = Quaternion(poses["turret"][1]) @ Vector((1, 0, 0))
        loc = tgt + d * 0.36 + Vector((-0.10, 0.0, 0.10))
        lens, fst = 50, 8.0
    elif a.view == "wrist":
        tgt = w("wrist", (0, 0, 15))
        d = Quaternion(poses["wrist"][1]) @ Vector((1, 0, 0))
        loc = tgt + d * 0.30 + Vector((0.0, 0.0, 0.07))
        lens, fst = 55, 8.0
    else:  # exploded shoulder: look across the J2 axis
        tgt = w("turret", (0, 45, 120))
        ax = Quaternion(poses["turret"][1]) @ Vector((0, 1, 0))
        side = ax.cross(Vector((0, 0, 1))).normalized()
        loc = tgt + side * 0.48 + ax * 0.10 + Vector((0, 0, 0.16))
        lens, fst = 45, None
    RB.camera(loc, tgt, lens=lens, fstop=fst)
    RB.studio(mats, tgt - loc)
    sc.cycles.samples = a.samples
    sc.render.resolution_x, sc.render.resolution_y = (int(v) for v in a.res.split("x"))
    out = ROOT / "render" / "out" / f"mech_{a.view}{a.suffix}.png"
    sc.render.filepath = str(out)
    bpy.ops.render.render(write_still=True)
    print("wrote", out)


if __name__ == "__main__":
    main()
