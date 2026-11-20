"""Procedural 'detail' materials for close-up renders (Blender 5 node API).

Everything is generated in the shader, so no image textures are needed:
  printed()   PETG/PLA with fine layer lines (bump) + slight sheen
  sandpaper() grit noise bump + grain colour variation
  brushed()   anisotropic brushed steel
  rubber()    micro-texture, very matte
  wood()      workbench top (rings + grain)
and add_bevel(mat) for rounded-edge highlights on hard STL edges.
"""

from __future__ import annotations

import bpy


def _base(name):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    return m, nt, nt.nodes["Principled BSDF"]


def _link(nt, a, b):
    nt.links.new(a, b)


def add_bevel(m, radius=0.0006, bump_from=None):
    """Rounded-edge shading. If `bump_from` (a Bump node) exists, chain bevel into its normal."""
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    bev = nt.nodes.new("ShaderNodeBevel")
    bev.inputs["Radius"].default_value = radius
    bev.samples = 8
    if bump_from is not None:
        _link(nt, bev.outputs["Normal"], bump_from.inputs["Normal"])
    else:
        _link(nt, bev.outputs["Normal"], b.inputs["Normal"])
    return m


def _bump_from(nt, height_socket, strength, distance=0.0002):
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = strength
    bump.inputs["Distance"].default_value = distance
    _link(nt, height_socket, bump.inputs["Height"])
    _link(nt, bump.outputs["Normal"], nt.nodes["Principled BSDF"].inputs["Normal"])
    return bump


def printed(name, color, rough=0.45, layer_mm=0.16, strength=0.18, axis="Z"):
    m, nt, b = _base(name)
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    if "Coat Weight" in b.inputs:
        b.inputs["Coat Weight"].default_value = 0.08
    tc = nt.nodes.new("ShaderNodeTexCoord")
    wave = nt.nodes.new("ShaderNodeTexWave")
    wave.wave_type = "BANDS"
    wave.bands_direction = axis
    wave.inputs["Scale"].default_value = 1.0 / layer_mm  # object coords are in mm (STLs are scaled 0.001 on the object)
    wave.inputs["Distortion"].default_value = 0.3
    _link(nt, tc.outputs["Object"], wave.inputs["Vector"])
    bump = _bump_from(nt, wave.outputs["Fac"], strength, 0.00008)
    add_bevel(m, 0.0007, bump)
    return m


def sandpaper(name, color, grit_scale=4.0):
    """grit_scale = noise frequency per mm (4 ~ coarse 240 grit look, 7 ~ fine 600)."""
    m, nt, b = _base(name)
    b.inputs["Roughness"].default_value = 0.95
    tc = nt.nodes.new("ShaderNodeTexCoord")
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = grit_scale
    noise.inputs["Detail"].default_value = 2.0
    _link(nt, tc.outputs["Object"], noise.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (color[0] * 0.55, color[1] * 0.55, color[2] * 0.55, 1)
    ramp.color_ramp.elements[1].color = (*color, 1)
    _link(nt, noise.outputs["Fac"], ramp.inputs["Fac"])
    _link(nt, ramp.outputs["Color"], b.inputs["Base Color"])
    _bump_from(nt, noise.outputs["Fac"], 0.9, 0.00025)
    return m


def brushed(name, color=(0.78, 0.79, 0.81), rough=0.22):
    m, nt, b = _base(name)
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Metallic"].default_value = 1.0
    b.inputs["Roughness"].default_value = rough
    if "Anisotropic" in b.inputs:
        b.inputs["Anisotropic"].default_value = 0.7
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mapping = nt.nodes.new("ShaderNodeMapping")
    mapping.inputs["Scale"].default_value = (1.0, 1.0, 60.0)  # streaks along the part
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 3.0
    _link(nt, tc.outputs["Object"], mapping.inputs["Vector"])
    _link(nt, mapping.outputs["Vector"], noise.inputs["Vector"])
    bump = _bump_from(nt, noise.outputs["Fac"], 0.06, 0.00005)
    add_bevel(m, 0.0004, bump)
    return m


def rubber(name, color=(0.02, 0.02, 0.022)):
    m, nt, b = _base(name)
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = 0.9
    tc = nt.nodes.new("ShaderNodeTexCoord")
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 2.5
    _link(nt, tc.outputs["Object"], noise.inputs["Vector"])
    bump = _bump_from(nt, noise.outputs["Fac"], 0.25, 0.0001)
    add_bevel(m, 0.0006, bump)
    return m


def wood(name="bench"):
    m, nt, b = _base(name)
    b.inputs["Roughness"].default_value = 0.55
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mapping = nt.nodes.new("ShaderNodeMapping")
    mapping.inputs["Scale"].default_value = (2.0, 18.0, 2.0)
    wave = nt.nodes.new("ShaderNodeTexWave")
    wave.wave_type = "RINGS"
    wave.inputs["Scale"].default_value = 3.0
    wave.inputs["Distortion"].default_value = 6.0
    wave.inputs["Detail"].default_value = 3.0
    _link(nt, tc.outputs["Generated"], mapping.inputs["Vector"])
    _link(nt, mapping.outputs["Vector"], wave.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (0.36, 0.22, 0.12, 1)
    ramp.color_ramp.elements[1].color = (0.62, 0.43, 0.26, 1)
    _link(nt, wave.outputs["Fac"], ramp.inputs["Fac"])
    _link(nt, ramp.outputs["Color"], b.inputs["Base Color"])
    _bump_from(nt, wave.outputs["Fac"], 0.05, 0.0002)
    return m


def sky_world(strength=0.6):
    w = bpy.context.scene.world
    nt = w.node_tree
    try:
        sky = nt.nodes.new("ShaderNodeTexSky")
        sky.sun_elevation = 0.6
        sky.sun_rotation = 2.2
        _link(nt, sky.outputs["Color"], nt.nodes["Background"].inputs["Color"])
        nt.nodes["Background"].inputs["Strength"].default_value = strength
    except Exception:
        nt.nodes["Background"].inputs["Color"].default_value = (0.75, 0.78, 0.82, 1)
