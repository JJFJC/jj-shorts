"""Render the jewelry studio: Gantry Arm sanding a pendant in the servo vise, the cold-work row it
serves, and the hot zone behind the steel heat-shield wall.

    blender-free: python render/render_studio.py --view overview|hot|cold [--samples 128] [--res 2400x1500]

Reuses the Gantry Arm renderer (render_blender.py) and the procedural detail materials.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parent
GA = REPO / "gantry-arm"
sys.path[:0] = [str(GA / "render"), str(GA / "sim"), str(GA), str(GA / "software" / "lerobot_robot_gantry_arm")]

import bpy  # noqa: E402
from mathutils import Quaternion, Vector  # noqa: E402

import detail_materials as DM  # noqa: E402
import fk  # noqa: E402
import render_blender as RB  # noqa: E402
from lerobot_robot_gantry_arm.kinematics import Geometry, Kinematics, Pose  # noqa: E402

FIN = GA / "cad" / "out" / "finishing"
STL = GA / "cad" / "out" / "stl"
CELL = REPO / "candle-cell" / "cad" / "out" / "arm_cell"
STUDIO = HERE / "cad" / "out"
SKIP = ("gripper_", "tool_", "coupler_contact_pads", "coupler_tool_half")


def obj(path, pose, material, smooth=35):
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
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(smooth))
    except Exception:
        pass
    return o


def studio_materials(mats):
    P = RB.principled
    mats.update({
        "wood": DM.wood("bench_wood"),
        "wood_light": P("wood_light", (0.70, 0.52, 0.32), 0.6),
        "frame": P("frame", (0.03, 0.03, 0.035), 0.5, metal=0.6),
        "steel": DM.brushed("st_steel"),
        "steel_dark": DM.brushed("st_dark", (0.16, 0.165, 0.175), 0.35),
        "steel_mesh": P("mesh", (0.6, 0.6, 0.62), 0.4, metal=1.0),
        "glass": P("glass", (0.92, 0.95, 0.97), 0.02, transmission=1.0, ior=1.5),
        "ceramic_white": P("ceramic", (0.86, 0.85, 0.82), 0.25, coat=0.5),
        "liquid": P("pickle", (0.55, 0.75, 0.85), 0.05, transmission=0.9, ior=1.33),
        "plastic_pp": P("pp", (0.9, 0.9, 0.86), 0.45),
        "plastic_green": P("tumb", (0.10, 0.35, 0.22), 0.4, coat=0.3),
        "media": P("media", (0.75, 0.55, 0.40), 0.8),
        "dark": P("dark", (0.025, 0.025, 0.028), 0.45, spec=0.4),
        "led": P("ring_led", (1, 1, 1), 0.3, emission=(1.0, 0.97, 0.9), strength=12.0),
        "lcd": P("lcd", (0.02, 0.08, 0.05), 0.2, emission=(0.2, 1.0, 0.55), strength=2.5),
        "lcd_red": P("lcd_r", (0.08, 0.01, 0.01), 0.2, emission=(1.0, 0.15, 0.05), strength=3.0),
        "enamel_blue": P("en_blue", (0.05, 0.16, 0.42), 0.3, coat=0.6),
        "enamel_grey": P("en_grey", (0.30, 0.31, 0.32), 0.35, coat=0.5),
        "enamel_dark": P("en_dark", (0.07, 0.07, 0.075), 0.35, coat=0.5),
        "enamel_red": P("en_red", (0.62, 0.03, 0.02), 0.3, coat=0.7),
        "cloth": P("cloth", (0.92, 0.90, 0.84), 1.0),
        "felt": DM.rubber("st_felt", (0.86, 0.86, 0.83)),
        "acrylic_orange": P("acr_or", (1.0, 0.35, 0.02), 0.05, transmission=0.85, ior=1.49),
        "silver": P("silver", (0.97, 0.96, 0.94), 0.08, metal=1.0),
        "gold": P("gold", (1.0, 0.77, 0.36), 0.1, metal=1.0),
        "brass": P("brass2", (0.85, 0.64, 0.30), 0.25, metal=1.0),
        "rubber_red": P("rub_red", (0.55, 0.06, 0.05), 0.8),
        "wax_green": P("wax", (0.18, 0.55, 0.30), 0.35, coat=0.3),
        "firebrick": DM.sandpaper("brick", (0.80, 0.72, 0.58), 1.2),
        "flame": P("flame", (0.2, 0.4, 1.0), 0.5, emission=(0.25, 0.5, 1.0), strength=40.0),
        "velvet": P("velvet", (0.05, 0.05, 0.12), 1.0),
        "printer": P("printer", (0.025, 0.025, 0.028), 0.4),
        "plate": DM.brushed("plate", (0.15, 0.15, 0.16), 0.3),
    })
    for k in ("frame", "dark", "enamel_grey", "enamel_dark", "enamel_red", "enamel_blue", "ceramic_white"):
        DM.add_bevel(mats[k], 0.0012)
    return mats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", default="overview", choices=["overview", "hot", "cold", "top"])
    ap.add_argument("--samples", type=int, default=128)
    ap.add_argument("--res", default="2400x1500")
    ap.add_argument("--suffix", default="")
    a = ap.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:])

    man = json.loads((FIN / "manifest.json").read_text())
    tcp = next(m["tcp_mm"] for m in man if "tcp_mm" in m and m["tool"] == "sand")
    g = Geometry.load()
    k = Kinematics(Geometry(**{**g.__dict__, "tool": g.tool - 80.0 + tcp}))
    q = k.ik(Pose(6.0, 203.0, 70.0, 180.0, 90.0))
    rack = json.loads((GA / "software" / "lerobot_robot_gantry_arm" / "lerobot_robot_gantry_arm" / "assets" / "geometry.json").read_text())["rack"]

    sc = RB.reset()
    mats = studio_materials(RB.make_materials("white"))
    white_shell = DM.printed("d_shell", (0.82, 0.82, 0.80), 0.5, 0.16, 0.12)
    mats.update({
        "shell": white_shell,
        "cap": DM.printed("d_cap", (0.20, 0.21, 0.22), 0.5, 0.16, 0.12),
        "abrasive": DM.sandpaper("d_abr", (0.42, 0.26, 0.13), 4.0),
        "abrasive_fine": DM.sandpaper("d_abrf", (0.62, 0.52, 0.38), 7.0),
        "metal": DM.brushed("d_metal"),
        "screw": DM.brushed("d_screw", (0.06, 0.06, 0.065), 0.35),
        "motor": DM.brushed("d_motor", (0.05, 0.05, 0.055), 0.3),
        "rubber": DM.rubber("d_rubber"),
        "part": RB.principled("part_silver", (0.97, 0.96, 0.94), 0.18, metal=1.0),  # silver pendant blank
    })

    # --- robot + gantry
    poses = fk.body_poses(q)
    for m in json.loads((GA / "cad" / "out" / "manifest.json").read_text()):
        if m["body"] in poses and not m["name"].startswith(SKIP):
            obj(STL / f"{m['name']}.stl", poses[m["body"]], mats.get(m["role"], mats["shell"]))
        elif m["body"] == "world":
            obj(STL / f"{m['name']}.stl", ([0, 0, 0], [1, 0, 0, 0]), mats.get(m["role"], mats["shell"]))
    for m in man:
        if m.get("tool") == "sand":
            obj(FIN / f"{m['name']}.stl", poses["tool"], mats.get(m["role"], mats["shell"]))
    xs = {int(s): v for s, v in rack["slots"].items()}
    qd = Kinematics().ik(Pose(xs[1], rack["y"], rack["tcp_z"], 180.0, rack["roll_deg"]))
    dpos, dquat = fk.body_poses(qd)["tool"]
    for slot, key in ((1, "fine"), (2, "polish")):
        pos = [dpos[0] + xs[slot] - xs[1], dpos[1], dpos[2]]
        for m in man:
            if m.get("tool") == key:
                obj(FIN / f"{m['name']}.stl", (pos, dquat), mats.get(m["role"], mats["shell"]))
    for m in man:
        if m["body"] == "world":
            obj(FIN / f"{m['name']}.stl", ([0, 0, 0], [1, 0, 0, 0]), mats.get(m["role"], mats["shell"]))
    # --- P1S (pattern printer: castable wax / resin patterns)
    for n, r in (("p1s_shell", "printer"), ("p1s_plate", "plate")):
        if (CELL / f"{n}.stl").exists():
            obj(CELL / f"{n}.stl", ([0, 0, 0], [1, 0, 0, 0]), mats[r])
    # --- studio stations
    for m in json.loads((STUDIO / "manifest.json").read_text()):
        obj(STUDIO / f"{m['name']}.stl", ([0, 0, 0], [1, 0, 0, 0]), mats.get(m["role"], mats["shell"]))

    # --- lighting: sky + studio area lights; warm practical over the hot zone
    DM.sky_world(0.04)
    sc.view_settings.exposure = -0.3
    views = {
        "overview": (Vector((0.1, -0.05, 0.1)), Vector((-0.75, 2.25, 1.75)), 26, None),
        "cold": (Vector((-0.10, 0.33, 0.08)), Vector((0.55, 1.05, 0.55)), 32, None),
        "hot": (Vector((0.30, -0.42, 0.16)), Vector((0.35, -1.45, 0.75)), 30, None),
        "top": (Vector((0.0, 0.0, 0.0)), Vector((0.0, -0.001, 3.8)), 24, None),
    }
    tgt, off, lens, fst = views[a.view]
    loc = tgt + off
    RB.camera(loc, tgt, lens=lens, fstop=fst)
    RB.studio(mats, tgt - loc)
    bpy.ops.object.light_add(type="AREA", location=(0.35, -0.45, 1.25))
    L = bpy.context.object
    L.data.energy = 220
    L.data.size = 1.2
    L.data.color = (1.0, 0.9, 0.78)
    bpy.ops.object.light_add(type="POINT", location=(0.765, -0.44, 0.07))  # torch flame glow
    F = bpy.context.object
    F.data.energy = 6
    F.data.color = (0.4, 0.6, 1.0)
    F.data.shadow_soft_size = 0.01
    bpy.data.objects["cyclorama"].location.z = -0.76  # shop floor under the bench legs
    sc.cycles.samples = a.samples
    sc.render.resolution_x, sc.render.resolution_y = (int(v) for v in a.res.split("x"))
    out = HERE / "render" / "out" / f"studio_{a.view}{a.suffix}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    sc.render.filepath = str(out)
    bpy.ops.render.render(write_still=True)
    print("pose", {kk: round(v, 1) for kk, v in q.items()})
    print("wrote", out)


if __name__ == "__main__":
    main()
