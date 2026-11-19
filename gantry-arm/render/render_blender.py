"""Photoreal renders of the Gantry Arm with Blender (bpy) Cycles on CPU.

    python render/render_blender.py --finish graphite --view threequarter
    python render/render_blender.py --all                      # every finish x view

CPU use is capped (threads=3 of 4, nice'd by the caller) to stay under the 90% rule.
Parts come from cad/out/stl/ (built by cad/build_all.py), placed with the MuJoCo forward
kinematics in sim/fk.py, so the renders always match params.py.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Euler, Quaternion, Vector

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(ROOT), str(ROOT / "sim")]
import fk  # noqa: E402

STL = ROOT / "cad" / "out" / "stl"
OUT = ROOT / "render" / "out"

POSES = {
    "hero": dict(gantry_x=430.0, gantry_z=230.0, base_yaw=-12.0, shoulder_lift=38.0, elbow_flex=62.0, wrist_flex=58.0, wrist_roll=0.0),
    "side": dict(gantry_x=400.0, gantry_z=230.0, base_yaw=0.0, shoulder_lift=38.0, elbow_flex=62.0, wrist_flex=58.0, wrist_roll=0.0),
    "close": dict(gantry_x=400.0, gantry_z=230.0, base_yaw=0.0, shoulder_lift=40.0, elbow_flex=75.0, wrist_flex=40.0, wrist_roll=0.0),
}

# --------------------------------------------------------------------------- materials


def principled(name, color, rough=0.5, metal=0.0, coat=0.0, transmission=0.0, emission=None, strength=0.0, ior=1.45, spec=0.5):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["IOR"].default_value = ior
    if "Specular IOR Level" in b.inputs:
        b.inputs["Specular IOR Level"].default_value = spec
    if coat:
        b.inputs["Coat Weight"].default_value = coat
        b.inputs["Coat Roughness"].default_value = 0.15
    if transmission:
        b.inputs["Transmission Weight"].default_value = transmission
    if emission:
        b.inputs["Emission Color"].default_value = (*emission, 1)
        b.inputs["Emission Strength"].default_value = strength
    return m


FINISHES = {
    # matte graphite: shells dark graphite, caps a step lighter for a two-tone read
    "graphite": dict(shell=((0.030, 0.032, 0.036), 0.62), cap=((0.085, 0.088, 0.095), 0.55), cover=((0.030, 0.032, 0.036), 0.62)),
    # matte white with grey end caps
    "white": dict(shell=((0.80, 0.80, 0.78), 0.55), cap=((0.20, 0.21, 0.22), 0.55), cover=((0.80, 0.80, 0.78), 0.55)),
}


def make_materials(finish: str) -> dict:
    f = FINISHES[finish]
    return {
        "shell": principled("shell", f["shell"][0], f["shell"][1], spec=0.35),
        "cap": principled("cap", f["cap"][0], f["cap"][1], spec=0.4),
        "accent": principled("led", (0.05, 0.4, 0.9), 0.3, emission=(0.1, 0.55, 1.0), strength=9.0),
        "extrusion": principled("anodised", (0.018, 0.018, 0.02), 0.32, metal=0.85),
        "strip": principled("strip", (0.025, 0.025, 0.028), 0.55),
        "metal": principled("steel", (0.72, 0.73, 0.75), 0.22, metal=1.0),
        "rubber": principled("rubber", (0.012, 0.012, 0.012), 0.85, spec=0.2),
        "smoke": principled("smoke", (0.10, 0.10, 0.11), 0.06, transmission=1.0, ior=1.5),
        "lens": principled("lens", (0.005, 0.005, 0.01), 0.05, coat=1.0),
        "brass": principled("brass", (0.85, 0.64, 0.30), 0.25, metal=1.0),
        "estop_red": principled("estop_red", (0.6, 0.02, 0.02), 0.35),
        "estop_yellow": principled("estop_yellow", (0.9, 0.7, 0.0), 0.45),
        "belt": principled("belt", (0.01, 0.01, 0.01), 0.7),
        "floor": principled("floor", (0.30, 0.305, 0.31), 0.7),
    }


# --------------------------------------------------------------------------- scene


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.render.threads_mode = "FIXED"
    sc.render.threads = 3
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.use_denoising = True
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Medium High Contrast"
    sc.render.film_transparent = False
    world = bpy.data.worlds.new("w")
    sc.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (0.62, 0.64, 0.67, 1)
    bg.inputs["Strength"].default_value = 0.25
    return sc


def add_parts(mats: dict, pose: dict, explode: float = 0.0):
    manifest = json.loads((ROOT / "cad" / "out" / "manifest.json").read_text())
    poses = fk.body_poses(pose)
    objs = []
    for m in manifest:
        if m["body"] not in poses:
            continue
        path = STL / f"{m['name']}.stl"
        bpy.ops.wm.stl_import(filepath=str(path))
        o = bpy.context.selected_objects[0]
        o.name = m["name"]
        pos, quat = poses[m["body"]]
        q = Quaternion(quat)
        local_shift = Vector((0, 0, 0))
        if explode and m["body"] == "tool" and m["name"] in EXPLODE_TOOL_SIDE:
            local_shift = Vector((0, 0, explode))
        o.scale = (0.001, 0.001, 0.001)
        o.rotation_mode = "QUATERNION"
        o.rotation_quaternion = q
        o.location = Vector(pos) / 1000 + q @ (local_shift / 1000)
        o.data.materials.append(mats.get(m["role"], mats["shell"]))
        bpy.context.view_layer.objects.active = o
        try:
            bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
        except Exception:
            bpy.ops.object.shade_smooth()
        objs.append(o)
    return objs


EXPLODE_TOOL_SIDE = {
    "coupler_tool_half", "coupler_contact_pads", "gripper_housing", "gripper_smoke_cap", "gripper_pinion_rack",
    "gripper_fingers_vlink", "gripper_pivot_pins", "gripper_pads",
}


def cyclorama(mat, toward: Vector, radius=1.6, depth=4.0, width=14.0):
    """Seamless floor-to-wall sweep; the wall stands `depth` m beyond the scene opposite the camera."""
    import bmesh

    me = bpy.data.meshes.new("cyc")
    bm = bmesh.new()
    prof = [(-8.0, 0.0)]
    for i in range(0, 17):  # quarter circle from floor to wall
        a = math.radians(90 * i / 16)
        prof.append((depth - radius + radius * math.sin(a), radius - radius * math.cos(a)))
    prof.append((depth, 8.0))
    rows = []
    for x in (-width / 2, width / 2):
        rows.append([bm.verts.new((x, y, z)) for y, z in prof])
    for i in range(len(prof) - 1):
        bm.faces.new((rows[0][i], rows[1][i], rows[1][i + 1], rows[0][i + 1]))
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new("cyclorama", me)
    bpy.context.collection.objects.link(o)
    o.data.materials.append(mat)
    for f in o.data.polygons:
        f.use_smooth = True
    d = Vector((toward.x, toward.y, 0)).normalized()
    o.rotation_euler = (0, 0, math.atan2(-d.x, d.y))
    return o


def studio(mats, cam_dir: Vector):
    cyclorama(mats["floor"], cam_dir)

    def area(name, loc, size, energy, color=(1, 1, 1), target=(0, 0.1, 0.45)):
        bpy.ops.object.light_add(type="AREA", location=loc)
        L = bpy.context.object
        L.name = name
        L.data.size = size
        L.data.energy = energy
        L.data.color = color
        L.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()

    # lights placed relative to the camera direction so every view gets key / fill / rim
    d = Vector((cam_dir.x, cam_dir.y, 0)).normalized()  # from camera toward the scene
    side = Vector((-d.y, d.x, 0))
    c = Vector((0, 0.1, 0.45))
    area("key", c - d * 2.2 + side * 1.8 + Vector((0, 0, 2.0)), 2.4, 380)
    area("fill", c - d * 2.0 - side * 2.4 + Vector((0, 0, 0.8)), 3.0, 120, (0.92, 0.95, 1.0))
    area("rim", c + d * 2.0 + side * 0.6 + Vector((0, 0, 2.0)), 1.8, 420, (1.0, 0.97, 0.92))
    area("top", c + Vector((0, 0, 2.8)), 3.0, 110)


def camera(loc, target, lens=50, fstop=None, focus=None):
    bpy.ops.object.camera_add(location=loc)
    cam = bpy.context.object
    d = Vector(target) - Vector(loc)
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = lens
    cam.data.sensor_width = 36
    if fstop:
        cam.data.dof.use_dof = True
        cam.data.dof.aperture_fstop = fstop
        cam.data.dof.focus_distance = focus or d.length
    bpy.context.scene.camera = cam
    return cam


def tcp_world(pose):
    import mujoco
    import model as M

    m = mujoco.MjModel.from_xml_string(M.to_mjcf(0.0))
    d = mujoco.MjData(m)
    for n, v in pose.items():
        j = m.joint(n)
        d.qpos[m.jnt_qposadr[j.id]] = v / 1000 if m.jnt_type[j.id] == mujoco.mjtJoint.mjJNT_SLIDE else math.radians(v)
    mujoco.mj_kinematics(m, d)
    return Vector(d.site("tcp").xpos), Vector(d.xpos[m.body("tool").id]), d


VIEWS = {
    # name: (pose, camera fn)
    "threequarter": "hero",
    "side": "side",
    "gripper": "close",
    "coupler": "close",
}


def render(finish: str, view: str, samples: int, res: tuple, suffix: str = ""):
    sc = reset()
    mats = make_materials(finish)
    pose = POSES[VIEWS[view]]
    add_parts(mats, pose, explode=22.0 if view == "coupler" else 0.0)
    tcp, tool, _ = tcp_world(pose)
    if view == "threequarter":
        tgt = Vector((-0.08, 0.10, 0.40))
        loc = tgt + Vector((math.cos(math.radians(118)) * 2.15, math.sin(math.radians(118)) * 2.15, 0.78))
        camera(loc, tgt, lens=42)
    elif view == "side":
        tgt = Vector((0.0, 0.16, 0.47))
        loc = Vector((-2.05, 0.16, 0.58))
        camera(loc, tgt, lens=45)
    elif view == "gripper":
        tgt = tool + (tcp - tool) * 0.55
        loc = tgt + Vector((0.36, 0.22, -0.06)).normalized() * 0.55
        camera(loc, tgt, lens=85, fstop=8.0)
    else:  # coupler, exploded 22 mm
        tgt = tool + Vector((0, 0, -0.004))
        tgt = tool + (tcp - tool) * 0.42
        loc = tgt + Vector((0.36, 0.22, 0.05)).normalized() * 0.62
        camera(loc, tgt, lens=85, fstop=8.0)
    studio(mats, tgt - loc)
    sc.cycles.samples = samples
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{finish}_{view}{suffix}.png"
    sc.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    print("wrote", path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--finish", default="graphite", choices=list(FINISHES))
    ap.add_argument("--view", default="threequarter", choices=list(VIEWS))
    ap.add_argument("--samples", type=int, default=128)
    ap.add_argument("--res", default="1600x1100")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--suffix", default="")
    a = ap.parse_args()
    res = tuple(int(x) for x in a.res.split("x"))
    jobs = [(f, v) for f in FINISHES for v in ("threequarter", "side", "gripper")] + [("graphite", "coupler"), ("white", "coupler")] if a.all else [(a.finish, a.view)]
    for f, v in jobs:
        render(f, v, a.samples, res, a.suffix)
