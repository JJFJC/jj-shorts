"""Render the wall workshop: Gantry Arm on a 1.8 x 0.6 m bench, pegboard wall, hot-work cart.

    python render/render_workshop.py --view overview|robot|wall|hot|seat [--samples 96] [--res 2400x1500]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parent
GA = REPO / "gantry-arm"
sys.path[:0] = [str(REPO / "jewelry-studio" / "render"), str(GA / "render"), str(GA / "sim"), str(GA), str(GA / "software" / "lerobot_robot_gantry_arm")]

import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402

import detail_materials as DM  # noqa: E402
import fk  # noqa: E402
import render_blender as RB  # noqa: E402
from lerobot_robot_gantry_arm.kinematics import Geometry, Kinematics, Pose  # noqa: E402
from render_studio import obj, studio_materials  # noqa: E402

FIN = GA / "cad" / "out" / "finishing"
STL = GA / "cad" / "out" / "stl"
CELL = REPO / "candle-cell" / "cad" / "out" / "arm_cell"
WS = HERE / "cad" / "out"
SKIP = ("gripper_", "tool_", "coupler_contact_pads", "coupler_tool_half")
# P1S turned 180 deg about z so its door faces the rail from the right-hand end of the bench
P1S_POSE = ([-129.0, 529.5, 0.0], [0.0, 0.0, 0.0, 1.0])


def pegboard_material():
    m = bpy.data.materials.new("pegboard")
    m.use_nodes = True
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    b.inputs["Roughness"].default_value = 0.75
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (1 / 25.4, 0.0, 1 / 25.4)  # 1 in hole pitch, object coords in mm
    fr = nt.nodes.new("ShaderNodeVectorMath")
    fr.operation = "FRACTION"
    sub = nt.nodes.new("ShaderNodeVectorMath")
    sub.operation = "SUBTRACT"
    sub.inputs[1].default_value = (0.5, 0.0, 0.5)
    ln = nt.nodes.new("ShaderNodeVectorMath")
    ln.operation = "LENGTH"
    lt = nt.nodes.new("ShaderNodeMath")
    lt.operation = "LESS_THAN"
    lt.inputs[1].default_value = 0.13
    mix = nt.nodes.new("ShaderNodeMix")
    mix.data_type = "RGBA"
    mix.inputs["A"].default_value = (0.42, 0.30, 0.19, 1)  # tempered hardboard
    mix.inputs["B"].default_value = (0.01, 0.008, 0.006, 1)
    L = nt.links.new
    L(tc.outputs["Object"], mp.inputs["Vector"])
    L(mp.outputs["Vector"], fr.inputs[0])
    L(fr.outputs["Vector"], sub.inputs[0])
    L(sub.outputs["Vector"], ln.inputs[0])
    L(ln.outputs["Value"], lt.inputs[0])
    L(lt.outputs["Value"], mix.inputs["Factor"])
    L(mix.outputs["Result"], b.inputs["Base Color"])
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.6
    bump.inputs["Distance"].default_value = 0.002
    bump.invert = True
    L(lt.outputs["Value"], bump.inputs["Height"])
    L(bump.outputs["Normal"], b.inputs["Normal"])
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", default="overview", choices=["overview", "robot", "wall", "hot", "seat"])
    ap.add_argument("--samples", type=int, default=96)
    ap.add_argument("--res", default="2400x1500")
    ap.add_argument("--suffix", default="")
    a = ap.parse_args()

    man = json.loads((FIN / "manifest.json").read_text())
    tcp = next(m["tcp_mm"] for m in man if "tcp_mm" in m and m["tool"] == "sand")
    g = Geometry.load()
    k = Kinematics(Geometry(**{**g.__dict__, "tool": g.tool - 80.0 + tcp}))
    q = k.ik(Pose(6.0, 203.0, 70.0, 180.0, 90.0))  # sanding a part in the servo vise
    rack = json.loads((GA / "software" / "lerobot_robot_gantry_arm" / "lerobot_robot_gantry_arm" / "assets" / "geometry.json").read_text())["rack"]

    sc = RB.reset()
    mats = studio_materials(RB.make_materials("white"))
    P = RB.principled
    mats.update({
        "shell": DM.printed("d_shell", (0.82, 0.82, 0.80), 0.5, 0.16, 0.12),
        "cap": DM.printed("d_cap", (0.20, 0.21, 0.22), 0.5, 0.16, 0.12),
        "abrasive": DM.sandpaper("d_abr", (0.42, 0.26, 0.13), 4.0),
        "abrasive_fine": DM.sandpaper("d_abrf", (0.62, 0.52, 0.38), 7.0),
        "metal": DM.brushed("d_metal"),
        "screw": DM.brushed("d_screw", (0.06, 0.06, 0.065), 0.35),
        "motor": DM.brushed("d_motor", (0.05, 0.05, 0.055), 0.3),
        "rubber": DM.rubber("d_rubber"),
        "part": P("part_orange_v", (0.85, 0.30, 0.04), 0.4),  # printed part being finished
        "pegboard": pegboard_material(),
        "wall_paint": P("wall", (0.50, 0.52, 0.53), 0.85),
        "cement_board": P("cement", (0.45, 0.45, 0.43), 0.9),
        "bin_blue": P("bin_b", (0.08, 0.25, 0.60), 0.45),
        "bin_red": P("bin_r", (0.65, 0.08, 0.05), 0.45),
        "bin_yellow": P("bin_y", (0.90, 0.62, 0.05), 0.45),
        "mat_green": P("mat", (0.05, 0.28, 0.16), 0.7),
        "part_orange": P("part_o", (0.85, 0.30, 0.04), 0.4),
        "filament_orange": P("fil_o", (0.85, 0.30, 0.04), 0.35),
        "handle_red": P("h_red", (0.6, 0.04, 0.03), 0.4),
        "handle_yellow": P("h_yel", (0.92, 0.65, 0.04), 0.4),
        "handle_blue": P("h_blue", (0.05, 0.18, 0.5), 0.4),
        "handle_green": P("h_green", (0.10, 0.45, 0.15), 0.4),
        "glass_tint": P("tint", (0.6, 0.7, 0.75), 0.05, transmission=0.8),
        "copper": P("copper", (0.95, 0.55, 0.38), 0.25, metal=1.0),
        "paper": P("paper", (0.93, 0.92, 0.88), 0.9),
        "bottle_white": P("bot_w", (0.9, 0.9, 0.88), 0.35),
        "bottle_amber": P("bot_a", (0.45, 0.20, 0.03), 0.1, transmission=0.6),
        "bottle_blue": P("bot_b", (0.1, 0.25, 0.65), 0.35),
        "enamel_white": P("en_white", (0.82, 0.82, 0.80), 0.35, coat=0.3),
        "leather": P("leather", (0.30, 0.17, 0.08), 0.65),
        "led": P("ledbar", (1, 1, 1), 0.3, emission=(1.0, 0.96, 0.9), strength=8.0),
    })

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
    dpos, dquat = fk.body_poses(Kinematics().ik(Pose(xs[1], rack["y"], rack["tcp_z"], 180.0, rack["roll_deg"])))["tool"]
    for slot, key in ((1, "fine"), (2, "polish")):
        for m in man:
            if m.get("tool") == key:
                obj(FIN / f"{m['name']}.stl", ([dpos[0] + xs[slot] - xs[1], dpos[1], dpos[2]], dquat), mats.get(m["role"], mats["shell"]))
    for m in man:
        if m["body"] == "world":
            obj(FIN / f"{m['name']}.stl", ([0, 0, 0], [1, 0, 0, 0]), mats.get(m["role"], mats["shell"]))
    for n, r in (("p1s_shell", "printer"), ("p1s_plate", "plate")):
        obj(CELL / f"{n}.stl", P1S_POSE, mats[r])
    for m in json.loads((WS / "manifest.json").read_text()):
        obj(WS / f"{m['name']}.stl", ([0, 0, 0], [1, 0, 0, 0]), mats.get(m["role"], mats["shell"]))

    DM.sky_world(0.03)
    sc.view_settings.exposure = -1.0
    views = {
        "overview": (Vector((-0.30, 0.05, 0.35)), Vector((0.95, 2.75, 1.05)), 24),
        "robot": (Vector((-0.12, 0.22, 0.12)), Vector((0.42, 0.95, 0.48)), 30),
        "wall": (Vector((0.0, -0.15, 0.80)), Vector((0.35, 2.05, 0.05)), 30),
        "hot": (Vector((-1.25, 0.05, 0.10)), Vector((0.55, 1.40, 0.55)), 30),
        "seat": (Vector((-0.74, 0.30, -0.06)), Vector((0.30, 0.78, 0.42)), 32),
    }
    tgt, off, lens = views[a.view]
    loc = tgt + off
    RB.camera(loc, tgt, lens=lens)
    RB.studio(mats, tgt - loc)
    bpy.data.objects["cyclorama"].location.z = -0.76
    for name, loc_, size, energy in (("ceiling", (-0.3, 0.6, 2.1), 2.6, 520), ("cart_light", (-1.25, 0.7, 1.2), 0.8, 90)):
        bpy.ops.object.light_add(type="AREA", location=loc_)
        L = bpy.context.object
        L.name, L.data.size, L.data.energy = name, size, energy
        L.data.color = (1.0, 0.95, 0.88)
    bpy.ops.object.light_add(type="POINT", location=(-1.15, 0.07, 0.03))  # torch flame glow
    F = bpy.context.object
    F.data.energy, F.data.color, F.data.shadow_soft_size = 5, (0.4, 0.6, 1.0), 0.01
    sc.cycles.samples = a.samples
    sc.render.resolution_x, sc.render.resolution_y = (int(v) for v in a.res.split("x"))
    out = HERE / "render" / "out" / f"workshop_{a.view}{a.suffix}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    sc.render.filepath = str(out)
    bpy.ops.render.render(write_still=True)
    print("wrote", out)


if __name__ == "__main__":
    main()
