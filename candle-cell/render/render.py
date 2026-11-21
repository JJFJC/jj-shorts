"""Blender renders of the candle cell: overview of the carousel + exploded mold.

    python render/render.py --view cell|mold [--samples 96] [--res 1600x1100]
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


def mat(name, color, rough=0.5, metal=0.0, transmission=0.0, emission=None, strength=0.0, sss=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if transmission:
        b.inputs["Transmission Weight"].default_value = transmission
    if sss:
        b.inputs["Subsurface Weight"].default_value = sss
        b.inputs["Subsurface Radius"].default_value = (0.004, 0.003, 0.002)
    if emission:
        b.inputs["Emission Color"].default_value = (*emission, 1)
        b.inputs["Emission Strength"].default_value = strength
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
    w.node_tree.nodes["Background"].inputs["Color"].default_value = (0.75, 0.76, 0.78, 1)
    w.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.3
    return sc


MATS = {}


def materials():
    MATS.update({
        "frame": mat("frame", (0.05, 0.055, 0.06), 0.55),
        "disc": mat("disc", (0.62, 0.63, 0.65), 0.35, metal=0.9),
        "dark": mat("dark", (0.015, 0.015, 0.018), 0.4),
        "steel": mat("steel", (0.70, 0.71, 0.72), 0.25, metal=1.0),
        "mold": mat("mold", (0.05, 0.42, 0.48), 0.35),  # teal PETG
        "mold2": mat("mold2", (0.03, 0.22, 0.26), 0.4),
        "wax": mat("wax", (0.93, 0.89, 0.80), 0.55, sss=0.4),
        "wax_liquid": mat("wax_liquid", (0.95, 0.78, 0.45), 0.08, transmission=0.35),
        "cover": mat("cover", (0.55, 0.6, 0.65), 0.1, transmission=0.85),
        "accent": mat("accent", (0.85, 0.35, 0.08), 0.45),
        "estop_red": mat("estop_red", (0.6, 0.02, 0.02), 0.35),
    })


def load(sub, offsets=None):
    d = ROOT / "cad" / "out" / sub
    for m in json.loads((d / "manifest.json").read_text()):
        bpy.ops.wm.stl_import(filepath=str(d / f"{m['name']}.stl"))
        o = bpy.context.selected_objects[0]
        o.scale = (0.001, 0.001, 0.001)
        if offsets and m["name"] in offsets:
            o.location = Vector(offsets[m["name"]]) / 1000
        o.data.materials.append(MATS.get(m["role"], MATS["frame"]))
        bpy.context.view_layer.objects.active = o
        try:
            bpy.ops.object.shade_smooth_by_angle(angle=math.radians(30))
        except Exception:
            pass


def studio(center, scale=1.0):
    bpy.ops.mesh.primitive_plane_add(size=30, location=(0, 0, -0.0005))
    bpy.context.object.data.materials.append(mat("floor", (0.30, 0.31, 0.32), 0.8))
    c = Vector(center)
    for loc, size, e, col in (((-1.6, -1.8, 2.2), 2.5, 380, (1.0, 0.96, 0.9)), ((1.8, -0.8, 1.4), 2.5, 140, (0.9, 0.94, 1.0)), ((0.5, 2.0, 2.0), 2.0, 260, (1, 1, 1))):
        bpy.ops.object.light_add(type="AREA", location=(c + Vector(loc) * scale))
        L = bpy.context.object
        L.data.size, L.data.energy, L.data.color = size * scale, e * scale * scale, col
        L.rotation_euler = (c - L.location).to_track_quat("-Z", "Y").to_euler()


def camera(loc, target, lens=45, fstop=None):
    bpy.ops.object.camera_add(location=loc)
    cam = bpy.context.object
    cam.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = lens
    if fstop:
        cam.data.dof.use_dof = True
        cam.data.dof.aperture_fstop = fstop
        cam.data.dof.focus_distance = (Vector(target) - Vector(loc)).length
    bpy.context.scene.camera = cam


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", choices=["cell", "mold"], default="cell")
    ap.add_argument("--samples", type=int, default=96)
    ap.add_argument("--res", default="1600x1100")
    ap.add_argument("--suffix", default="")
    a = ap.parse_args()
    setup(a.samples, tuple(int(v) for v in a.res.split("x")))
    materials()
    if a.view == "cell":
        load("cell")
        studio((0, 0, 0.2))
        camera((0.80, -1.20, 1.15), (-0.03, 0.02, 0.15), lens=36)
    else:
        # exploded mold: halves apart, piston below, wick bar above, a finished candle beside
        off = {"mold_half_a": (-45, 0, 0), "mold_half_b": (45, 0, 0), "piston": (0, -95, 0), "wick_bar": (0, 0, 40)}
        load("mold", off)
        bpy.ops.wm.stl_import(filepath=str(ROOT / "cad" / "out" / "cell" / "out_candle_00.stl"))
        o = bpy.context.selected_objects[0]
        o.scale = (0.001, 0.001, 0.001)
        o.location = Vector((0.61, -0.03, 0.0))  # tray candle (-460, 30) -> beside the mold (+150, 0)
        o.data.materials.append(MATS["wax"])
        studio((0, 0, 0.05), scale=0.35)
        camera((0.32, -0.48, 0.30), (0.05, -0.02, 0.03), lens=50, fstop=8)
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"candle_{a.view}{a.suffix}.png"
    bpy.context.scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    print("wrote", path)
