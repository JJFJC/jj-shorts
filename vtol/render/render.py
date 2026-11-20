"""Blender (bpy) Cycles renders of KESTREL from cad/out/<mode>/*.stl.

    python render/render.py --view hover34|cruise34|top [--samples 96] [--res 1600x1000]

CPU threads capped at 3 (of 4) for the 90 % CPU rule.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "render" / "out"
LIFT = 0.55  # m: aircraft floats this high above the floor in the renders


def mat(name, color, rough=0.5, metal=0.0, emission=None, strength=0.0, coat=0.0, alpha=1.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if coat:
        b.inputs["Coat Weight"].default_value = coat
    if emission:
        b.inputs["Emission Color"].default_value = (*emission, 1)
        b.inputs["Emission Strength"].default_value = strength
    if alpha < 1:
        b.inputs["Alpha"].default_value = alpha
    return m


def setup(samples, res):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.render.threads_mode = "FIXED"
    sc.render.threads = 3
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Medium High Contrast"
    sc.render.resolution_x, sc.render.resolution_y = res
    w = bpy.data.worlds.new("w")
    sc.world = w
    w.use_nodes = True
    w.node_tree.nodes["Background"].inputs["Color"].default_value = (0.70, 0.74, 0.80, 1)
    w.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.35
    return sc


def load(mode):
    mats = {
        "shell": mat("shell", (0.80, 0.80, 0.78), 0.45, coat=0.2),
        "cap": mat("cap", (0.16, 0.17, 0.18), 0.5),
        "accent2": mat("accent2", (0.14, 0.15, 0.16), 0.45),
        "motor": mat("motor", (0.05, 0.05, 0.055), 0.3, metal=0.8),
        "prop": mat("prop", (0.03, 0.03, 0.035), 0.35),
        "lens": mat("lens", (0.004, 0.004, 0.01), 0.05, coat=1.0),
        "nav_red": mat("nav_red", (0.6, 0.02, 0.02), 0.3, emission=(1.0, 0.05, 0.05), strength=25),
        "nav_green": mat("nav_green", (0.02, 0.6, 0.1), 0.3, emission=(0.1, 1.0, 0.25), strength=25),
    }
    d = ROOT / "cad" / "out" / mode
    for m in json.loads((d / "manifest.json").read_text()):
        bpy.ops.wm.stl_import(filepath=str(d / f"{m['name']}.stl"))
        o = bpy.context.selected_objects[0]
        o.scale = (0.001, 0.001, 0.001)
        o.location = (0, 0, LIFT)
        o.data.materials.append(mats.get(m["role"], mats["shell"]))
        bpy.context.view_layer.objects.active = o
        try:
            bpy.ops.object.shade_smooth_by_angle(angle=math.radians(30))
        except Exception:
            bpy.ops.object.shade_smooth()


def studio():
    bpy.ops.mesh.primitive_plane_add(size=40, location=(0, 0, 0))
    bpy.context.object.data.materials.append(mat("floor", (0.22, 0.235, 0.25), 0.85))
    for name, loc, size, energy, col in (
        ("key", (-2.5, 2.5, 3.5), 3.0, 800, (1.0, 0.97, 0.93)),
        ("fill", (2.5, -2.0, 2.0), 3.5, 250, (0.9, 0.94, 1.0)),
        ("rim", (3.0, 2.0, 2.5), 2.0, 450, (1.0, 1.0, 1.0)),
    ):
        bpy.ops.object.light_add(type="AREA", location=loc)
        L = bpy.context.object
        L.data.size, L.data.energy, L.data.color = size, energy, col
        L.rotation_euler = (Vector((0, 0, LIFT)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()


def camera(loc, target, lens=50, ortho=None):
    bpy.ops.object.camera_add(location=loc)
    cam = bpy.context.object
    cam.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = lens
    if ortho:
        cam.data.type = "ORTHO"
        cam.data.ortho_scale = ortho
    bpy.context.scene.camera = cam


VIEWS = {
    # name: (mode, camera location, target, lens, ortho)
    "hover34": ("hover", (-1.55, -1.35, LIFT + 0.85), (-0.03, 0.0, LIFT), 45, None),
    "cruise34": ("cruise", (-1.55, 1.30, LIFT + 0.50), (-0.02, 0.0, LIFT - 0.02), 45, None),
    "top": ("cruise", (0.02, 0.0, LIFT + 3.0), (0.02, 0.0, LIFT), 50, 1.18),
}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", choices=list(VIEWS), default="hover34")
    ap.add_argument("--samples", type=int, default=96)
    ap.add_argument("--res", default="1600x1000")
    ap.add_argument("--suffix", default="")
    a = ap.parse_args()
    res = tuple(int(v) for v in a.res.split("x"))
    mode, loc, tgt, lens, ortho = VIEWS[a.view]
    setup(a.samples, res)
    load(mode)
    studio()
    camera(loc, tgt, lens, ortho)
    if a.view == "top":
        bpy.context.scene.camera.rotation_euler = (0, 0, math.radians(90))  # nose up the page
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"kestrel_{a.view}{a.suffix}.png"
    bpy.context.scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    print("wrote", path)
