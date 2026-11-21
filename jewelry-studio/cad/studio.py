"""Jewelry studio around the Gantry Arm: every station as CAD (concept-level detail).

    python cad/studio.py  -> cad/out/*.stl + manifest.json + layout.json

World frame = Gantry Arm world (X along the rail, +Y out over the work area, z up, bench top z=0).
Front row = cold work the robot serves. Behind the steel heat-shield wall (y < -230) = hot zone,
run by a person; the arm cannot reach there (by design).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import cadquery as cq
from cadquery import Vector as V

OUT = Path(__file__).resolve().parent / "out"


def W(s):
    return cq.Workplane("XY").add(s)


def box(x0, x1, y0, y1, z0, z1):
    return W(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0)))


def cbox(cx, cy, w, d, z0, h, r=0.0):
    b = box(cx - w / 2, cx + w / 2, cy - d / 2, cy + d / 2, z0, z0 + h)
    if r:
        try:
            b = b.edges("|Z").fillet(r)
        except Exception:
            pass
    return b


def cyl(r, p0, p1):
    d = V(*p1) - V(*p0)
    return W(cq.Solid.makeCylinder(r, d.Length, V(*p0), d.normalized()))


def cone(r0, r1, p0, p1):
    d = V(*p1) - V(*p0)
    return W(cq.Solid.makeCone(r0, r1, d.Length, V(*p0), d.normalized()))


def torus(R, r, c, axis=(0, 0, 1)):
    return W(cq.Solid.makeTorus(R, r, V(*c), V(*axis)))


def fil(wp, r, sel=None):
    try:
        return (wp.edges(sel) if sel else wp.edges()).fillet(r)
    except Exception:
        return wp


def sphere(r, c):
    return W(cq.Solid.makeSphere(r, V(*c)))


LAYOUT = {
    # cold row (robot): name -> (x, y)
    "pickle": (-540.0, 520.0),
    "ultrasonic": (-380.0, 440.0),
    "tumbler": (-200.0, 455.0),
    "inspection": (-40.0, 440.0),
    "scale": (95.0, 450.0),
    "polisher": (640.0, 330.0),
    "laser": (1010.0, 300.0),
    # hot zone (person)
    "invest": (-300.0, -470.0),
    "kiln": (0.0, -470.0),
    "furnace": (290.0, -460.0),
    "caster": (520.0, -460.0),
    "torch": (800.0, -440.0),
    # hand bench
    "bench_pin": (-1220.0, 560.0),
}


def build():
    P = []

    def add(name, role, shape):
        P.append((name, role, shape))

    # ---------------------------------------------------------- bench + heat-shield wall
    add("bench", "wood", box(-1350, 1350, -720, 760, -40, 0))
    add("bench_legs", "frame", box(-1330, -1290, -700, 740, -760, -40).union(box(1290, 1330, -700, 740, -760, -40)))
    wall = box(-700, 1100, -250, -244, 0, 520)
    wall = wall.cut(box(-550, 950, -260, -230, 260, 470))
    add("heat_shield_wall", "steel_dark", wall)
    add("wall_window", "glass", box(-550, 950, -248, -246, 260, 470))

    # ---------------------------------------------------------- cold row
    x, y = LAYOUT["pickle"]  # small slow-cooker with citric pickle + PP basket
    pot = fil(cyl(85, (x, y, 0), (x, y, 120)), 12, "%CIRCLE").cut(cyl(75, (x, y, 20), (x, y, 125)))
    add("pickle_pot", "ceramic_white", pot)
    add("pickle_liquid", "liquid", cyl(75, (x, y, 20), (x, y, 95)))
    add("pickle_basket", "plastic_pp", cyl(45, (x, y, 60), (x, y, 112)).cut(cyl(43, (x, y, 62), (x, y, 113))).union(box(x - 3, x + 3, y - 2, y + 2, 112, 150)))
    x, y = LAYOUT["ultrasonic"]  # stainless ultrasonic cleaner
    us = cbox(x, y, 170, 120, 0, 95, 8).cut(cbox(x, y, 150, 100, 30, 70, 4))
    add("ultrasonic_body", "steel", us)
    add("ultrasonic_panel", "dark", box(x - 60, x + 60, y - 62, y - 59, 15, 45))
    add("ultrasonic_basket", "steel_mesh", cbox(x, y, 130, 80, 40, 52, 3).cut(cbox(x, y, 124, 74, 43, 52, 2)))
    x, y = LAYOUT["tumbler"]  # vibratory tumbler: bowl on base with media
    base = fil(cyl(80, (x, y, 0), (x, y, 70)), 6, ">Z")
    bowl = fil(cyl(95, (x, y, 70), (x, y, 140)), 8, "%CIRCLE").cut(cyl(88, (x, y, 78), (x, y, 141))).cut(cyl(22, (x, y, 60), (x, y, 150)))
    add("tumbler_base", "dark", base)
    add("tumbler_bowl", "plastic_green", bowl.union(cyl(20, (x, y, 70), (x, y, 150)).cut(cyl(18, (x, y, 70), (x, y, 151)))))
    media = None
    for i in range(28):  # ceramic / plastic cone media visible in the bowl
        a = i * 2.4
        rr = 35 + (i % 7) * 7
        c = cone(4.5, 0.8, (x + rr * math.cos(a), y + rr * math.sin(a), 105), (x + rr * math.cos(a), y + rr * math.sin(a), 113))
        media = c if media is None else media.union(c)
    add("tumbler_media", "media", media)
    x, y = LAYOUT["inspection"]  # macro camera + ring light + turntable
    add("insp_base", "dark", cbox(x, y, 120, 120, 0, 18, 6))
    add("insp_turntable", "steel", cyl(40, (x, y, 18), (x, y, 26)))
    add("insp_post", "steel", cyl(8, (x, y + 50, 18), (x, y + 50, 260)))
    add("insp_arm", "steel", box(x - 6, x + 6, y - 2, y + 56, 230, 242))
    add("insp_camera", "dark", cyl(18, (x, y, 170), (x, y, 230)))
    add("insp_ringlight", "led", torus(30, 5, (x, y, 162)))
    add("insp_ring_on_table", "silver", torus(9, 1.6, (x, y, 30), (0, 1, 0)))
    x, y = LAYOUT["scale"]  # precision scale with draft shield
    add("scale_body", "steel_dark", cbox(x, y, 110, 150, 0, 35, 6))
    add("scale_pan", "steel", cyl(30, (x, y + 15, 35), (x, y + 15, 38)))
    add("scale_display", "lcd", box(x - 30, x + 30, y - 76, y - 74, 10, 28))
    add("scale_shield", "glass", cyl(45, (x, y + 15, 38), (x, y + 15, 110)).cut(cyl(43, (x, y + 15, 38), (x, y + 15, 111))))
    x, y = LAYOUT["polisher"]  # bench polisher: motor + 2 buffs + dust hoods
    add("polisher_base", "dark", cbox(x, y, 120, 170, 0, 20, 8))
    add("polisher_motor", "enamel_blue", cyl(55, (x - 70, y, 110), (x + 70, y, 110)).union(cyl(30, (x, y, 20), (x, y, 70))))
    add("polisher_shaft", "steel", cyl(6, (x - 160, y, 110), (x + 160, y, 110)))
    add("polisher_buff_l", "cloth", cyl(60, (x - 150, y, 110), (x - 110, y, 110)))
    add("polisher_buff_r", "felt", cyl(60, (x + 110, y, 110), (x + 150, y, 110)))
    for sgn in (-1, 1):
        hood = cyl(72, (x + sgn * 105, y, 110), (x + sgn * 158, y, 110)).cut(cyl(68, (x + sgn * 104, y, 110), (x + sgn * 159, y, 110))).cut(box(x - 200, x + 200, y - 200, y + 40, 0, 300))
        add(f"polisher_hood_{'l' if sgn < 0 else 'r'}", "steel_dark", hood)
    x, y = LAYOUT["laser"]  # enclosed diode laser engraver with orange window
    enc = cbox(x, y, 300, 280, 0, 220, 10).cut(cbox(x, y, 284, 264, 8, 214, 6))
    enc = enc.cut(box(x - 150, x - 120, y - 120, y + 120, 30, 200))  # loading side opening facing the robot (-X)
    add("laser_enclosure", "dark", enc)
    add("laser_window", "acrylic_orange", box(x - 130, x + 130, y - 141, y - 138, 60, 200))
    add("laser_gantry", "steel", box(x - 120, x + 120, y - 10, y + 10, 150, 160).union(cbox(x + 20, y, 40, 40, 120, 40)))
    add("laser_bed", "steel_mesh", cbox(x, y, 240, 220, 10, 6))
    add("laser_part", "silver", cyl(18, (x + 20, y, 16), (x + 20, y, 18)))

    # ---------------------------------------------------------- hot zone (behind the wall, person only)
    x, y = LAYOUT["invest"]  # vacuum investing bell + flask + wax tree
    add("invest_base", "enamel_grey", cbox(x, y, 220, 220, 0, 110, 10))
    add("invest_bell", "glass", fil(cyl(80, (x, y, 110), (x, y, 260)), 30, ">Z").cut(fil(cyl(76, (x, y, 110), (x, y, 256)), 26, ">Z")))
    add("invest_gauge", "steel", cyl(18, (x + 60, y - 111, 75), (x + 60, y - 115, 75)))
    tree_x, tree_y = x - 170, y + 40
    add("wax_tree_base", "rubber_red", cone(40, 20, (tree_x, tree_y, 0), (tree_x, tree_y, 20)))
    tree = cyl(4, (tree_x, tree_y, 20), (tree_x, tree_y, 150))
    for i in range(10):
        a = math.radians(i * 72 + (i // 5) * 36)
        zz = 50 + i * 10
        bx, by = tree_x + 26 * math.cos(a), tree_y + 26 * math.sin(a)
        tree = tree.union(cyl(1.5, (tree_x, tree_y, zz), (bx, by, zz + 14))).union(torus(9, 1.8, (bx + 6 * math.cos(a), by + 6 * math.sin(a), zz + 20), (math.cos(a), math.sin(a), 0)))
    add("wax_tree", "wax_green", tree)
    add("flask", "steel", cyl(45, (x + 170, y + 40, 0), (x + 170, y + 40, 100)).cut(cyl(43, (x + 170, y + 40, 2), (x + 170, y + 40, 101))))
    x, y = LAYOUT["kiln"]  # burnout kiln with digital controller
    k = cbox(x, y, 300, 300, 0, 380, 14)
    add("kiln_body", "enamel_grey", k)
    add("kiln_door", "enamel_dark", box(x - 110, x + 110, y + 150, y + 160, 60, 300))
    add("kiln_handle", "steel", box(x + 70, x + 85, y + 160, y + 172, 150, 230))
    add("kiln_controller", "dark", box(x - 90, x + 10, y + 150, y + 162, 310, 365))
    add("kiln_display", "lcd_red", box(x - 80, x - 20, y + 162, y + 164, 335, 355))
    add("kiln_vent", "steel", cyl(20, (x, y, 380), (x, y, 420)))
    x, y = LAYOUT["furnace"]  # electric melting furnace (crucible)
    add("furnace_body", "enamel_dark", fil(cyl(80, (x, y, 0), (x, y, 260)), 8, ">Z"))
    add("furnace_lid", "steel", cyl(55, (x, y, 260), (x, y, 280)).union(box(x - 4, x + 4, y - 60, y + 60, 280, 290)))
    add("furnace_control", "dark", cbox(x + 140, y + 40, 120, 90, 0, 90, 6))
    add("furnace_display", "lcd_red", box(x + 100, x + 160, y + 85, y + 87, 50, 75))
    add("crucible_tongs", "steel", box(x - 170, x - 160, y + 60, y + 70, 2, 8).union(box(x - 170, x - 40, y + 70, y + 78, 2, 8)))
    x, y = LAYOUT["caster"]  # vacuum casting machine (flask well + pump)
    add("caster_body", "enamel_grey", cbox(x, y, 220, 220, 0, 180, 10))
    add("caster_well", "steel_dark", cyl(55, (x, y, 180), (x, y, 200)).cut(cyl(47, (x, y, 185), (x, y, 201))))
    add("caster_gauge", "steel", cyl(20, (x - 70, y + 111, 130), (x - 70, y + 116, 130)))
    add("caster_pump", "dark", cbox(x, y - 160, 160, 90, 0, 110, 8))
    x, y = LAYOUT["torch"]  # FIXED torch station: steel bracket, fire-brick turntable, fume hood
    add("torch_station_base", "steel_dark", cbox(x, y, 320, 260, 0, 12, 6))
    for i in range(3):  # fire bricks (turntable)
        for j in range(2):
            add(f"firebrick_{i}{j}", "firebrick", cbox(x - 40 + j * 82 - 41, y - 60 + i * 42, 80, 40, 12 + 30, 30, 1))
    add("torch_turntable", "steel", cyl(90, (x - 40, y, 12), (x - 40, y, 42)))
    stand = box(x + 100, x + 112, y - 15, y + 15, 12, 260)
    bracket = box(x + 60, x + 112, y - 20, y + 20, 200, 214).union(box(x + 60, x + 72, y - 20, y + 20, 160, 214))
    add("torch_stand", "steel", stand.union(bracket))
    add("torch_clamp", "steel_dark", cyl(22, (x + 66, y, 170), (x + 66, y, 205)).cut(cyl(16, (x + 66, y, 160), (x + 66, y, 210))))
    torch = cyl(15, (x + 66, y, 130), (x + 66, y, 240)).union(cone(15, 5, (x + 66, y, 130), (x + 20, y, 100))).union(cyl(5, (x + 20, y, 100), (x - 10, y, 80)))
    add("torch_body", "brass", torch)
    add("torch_flame", "flame", cone(4.5, 0.5, (x - 10, y, 80), (x - 35, y, 64)))
    add("gas_canister", "enamel_red", fil(cyl(40, (x + 200, y + 60, 0), (x + 200, y + 60, 200)), 15, ">Z"))
    add("gas_solenoid", "brass", cbox(x + 150, y + 20, 30, 30, 200, 25))
    add("flame_sensor", "steel", cyl(4, (x - 70, y + 40, 42), (x - 40, y + 30, 75)))
    add("fume_hood", "steel_dark", box(x - 180, x + 180, y - 150, y + 150, 420, 426).union(box(x - 180, x + 180, y + 144, y + 150, 300, 426)).union(cyl(55, (x, y, 426), (x, y, 520))))
    add("fire_extinguisher", "enamel_red", fil(cyl(50, (1000, -330, 0), (1000, -330, 380)), 20, ">Z").union(box(990, 1010, -340, -320, 380, 420)))
    # ---------------------------------------------------------- hand bench + finished pieces
    x, y = LAYOUT["bench_pin"]
    add("bench_pin", "wood_light", box(x - 60, x + 60, y - 30, y + 90, 25, 40).cut(box(x - 15, x + 15, y + 50, y + 91, 20, 45)))
    add("ring_mandrel", "steel", cone(7, 16, (x + 110, y + 150, 15), (x + 110, y - 120, 15)).union(cyl(12, (x + 110, y + 150, 15), (x + 110, y + 190, 15))))
    tray = cbox(-860, 640, 160, 110, 0, 12, 6).cut(cbox(-700, 330, 150, 100, 3, 13, 4))
    add("finished_tray", "velvet", tray)
    for i in range(3):
        for j in range(2):
            add(f"ring_{i}{j}", "silver" if (i + j) % 2 == 0 else "gold", torus(10, 2.2, (-910 + i * 50, 615 + j * 50, 14), (0, 1, 0)))
    return P


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    man = []
    for name, role, shape in build():
        s = shape.val() if shape.size() == 1 else shape.combine().val()
        cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.08, angularTolerance=0.15)
        man.append(dict(name=name, role=role, valid=s.isValid()))
    (OUT / "manifest.json").write_text(json.dumps(man, indent=1))
    (OUT / "layout.json").write_text(json.dumps(LAYOUT, indent=1))
    print(len(man), "parts; invalid:", [m["name"] for m in man if not m["valid"]])


if __name__ == "__main__":
    main()
