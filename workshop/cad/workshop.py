"""Wall workshop around the Gantry Arm: 1.8 x 0.6 m bench against a wall, pegboard above,
hot gear on a separate steel rolling cart.

    python cad/workshop.py  -> cad/out/*.stl + manifest.json + layout.json + reach.json

World frame = Gantry Arm world: X along the rail, +Y away from the wall (toward you), z up,
bench top z = 0. The wall face is at y = WALL_Y.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "jewelry-studio" / "cad"))
from studio import box, cbox, cone, cyl, fil, torus  # noqa: E402

OUT = HERE / "out"

BENCH = dict(x0=-900.0, x1=900.0, y0=-190.0, y1=410.0, top=40.0, h=760.0)  # 1.8 x 0.6 m, top z=0
WALL_Y = BENCH["y0"]
PEG = dict(x0=-900.0, x1=900.0, z0=300.0, z1=1200.0, standoff=20.0, t=6.0)  # pegboard on furring strips
PEG_FACE = WALL_Y + PEG["standoff"] + PEG["t"]
CART = dict(x0=-1560.0, x1=-960.0, y0=-160.0, y1=290.0, top=-30.0, shelf=-480.0)  # 600 x 450 steel cart

# station centres (x, y) and the height the robot works at; checked with the arm IK in main()
STATIONS = {
    "vise": dict(xy=(0.0, 205.0), z=70.0, what="Servo vise: sand / polish / drill / deburr any small part"),
    "tool_rack": dict(xy=(250.0, 275.0), z=110.0, what="Tool rack: 3 compliant spindles (240, 600, polish); tool change"),
    "p1s_plate": dict(xy=(535.0, 220.0), z=69.0, what="P1S build-plate tab: unload prints"),
    "ultrasonic": dict(xy=(-600.0, 250.0), z=150.0, what="Ultrasonic cleaner: parts, jewelry, nozzles"),
    "tumbler": dict(xy=(-400.0, 300.0), z=170.0, what="Vibratory tumbler: deburr / polish batches"),
    "inspection": dict(xy=(-220.0, 330.0), z=45.0, what="Camera + turntable: QC photos, measuring"),
    "scale": dict(xy=(-80.0, 330.0), z=60.0, what="Scale: weigh parts, count screws, filament left"),
    "pickle": dict(xy=(120.0, 340.0), z=150.0, what="Pickle crock (citric): clean metal after soldering"),
}


def build():
    P = []

    def add(name, role, shape):
        P.append((name, role, shape))

    b = BENCH
    # ------------------------------------------------------------------ room + bench
    add("wall", "wall_paint", box(-1800, 1300, WALL_Y - 40, WALL_Y, -800, 1500))
    add("bench_top", "wood", box(b["x0"], b["x1"], b["y0"], b["y1"], -b["top"], 0))
    legs = None
    for x in (b["x0"] + 30, b["x1"] - 30):
        for y in (b["y0"] + 30, b["y1"] - 30):
            l = cbox(x, y, 50, 50, -b["h"], b["h"] - b["top"], 4)
            legs = l if legs is None else legs.union(l)
    legs = legs.union(box(b["x0"] + 10, b["x1"] - 10, b["y0"] + 40, b["y1"] - 40, -620, -600))  # lower shelf
    add("bench_frame", "frame", legs)
    add("bench_shelf_bins", "bin_blue", cbox(-300, 110, 300, 400, -600, 180, 6).union(cbox(250, 110, 300, 400, -600, 180, 6)))

    # ------------------------------------------------------------------ pegboard + furring
    pg = PEG
    add("pegboard", "pegboard", box(pg["x0"], pg["x1"], WALL_Y + pg["standoff"], PEG_FACE, pg["z0"], pg["z1"]))
    furr = None
    for z in (pg["z0"] + 20, (pg["z0"] + pg["z1"]) / 2, pg["z1"] - 20):
        f = box(pg["x0"], pg["x1"], WALL_Y, WALL_Y + pg["standoff"], z - 20, z + 20)
        furr = f if furr is None else furr.union(f)
    add("pegboard_furring", "wood_light", furr)
    _pegboard_tools(add)

    # ------------------------------------------------------------------ robot stations (cold work)
    x, y = STATIONS["ultrasonic"]["xy"]
    add("ultrasonic_body", "steel", cbox(x, y, 170, 120, 0, 95, 8).cut(cbox(x, y, 150, 100, 30, 70, 4)))
    add("ultrasonic_panel", "dark", box(x - 60, x + 60, y + 59, y + 62, 15, 45))
    add("ultrasonic_basket", "steel_mesh", cbox(x, y, 130, 80, 40, 52, 3).cut(cbox(x, y, 124, 74, 43, 52, 2)))
    x, y = STATIONS["tumbler"]["xy"]
    add("tumbler_base", "dark", fil(cyl(80, (x, y, 0), (x, y, 70)), 6, ">Z"))
    bowl = fil(cyl(95, (x, y, 70), (x, y, 140)), 8, "%CIRCLE").cut(cyl(88, (x, y, 78), (x, y, 141)))
    add("tumbler_bowl", "plastic_green", bowl.cut(cyl(20, (x, y, 60), (x, y, 150))).union(cyl(20, (x, y, 70), (x, y, 150)).cut(cyl(18, (x, y, 70), (x, y, 151)))))
    media = None
    for i in range(28):
        a, rr = i * 2.4, 35 + (i % 7) * 7
        c = cone(4.5, 0.8, (x + rr * math.cos(a), y + rr * math.sin(a), 105), (x + rr * math.cos(a), y + rr * math.sin(a), 113))
        media = c if media is None else media.union(c)
    add("tumbler_media", "media", media)
    x, y = STATIONS["inspection"]["xy"]
    add("insp_base", "dark", cbox(x, y, 120, 120, 0, 18, 6))
    add("insp_turntable", "steel", cyl(40, (x, y, 18), (x, y, 26)))
    add("insp_post", "steel", cyl(8, (x, y - 50, 18), (x, y - 50, 260)))
    add("insp_arm", "steel", box(x - 6, x + 6, y - 56, y + 2, 230, 242))
    add("insp_camera", "dark", cyl(18, (x, y, 170), (x, y, 230)))
    add("insp_ringlight", "led", torus(30, 5, (x, y, 162)))
    add("insp_part", "part_orange", cbox(x, y, 30, 22, 26, 14, 3))  # a printed bracket on the turntable
    x, y = STATIONS["scale"]["xy"]
    add("scale_body", "steel_dark", cbox(x, y, 110, 150, 0, 35, 6))
    add("scale_pan", "steel", cyl(30, (x, y - 15, 35), (x, y - 15, 38)))
    add("scale_display", "lcd", box(x - 30, x + 30, y + 74, y + 76, 10, 28))
    x, y = STATIONS["pickle"]["xy"]
    add("pickle_pot", "ceramic_white", fil(cyl(70, (x, y, 0), (x, y, 110)), 10, "%CIRCLE").cut(cyl(61, (x, y, 18), (x, y, 115))))
    add("pickle_liquid", "liquid", cyl(61, (x, y, 18), (x, y, 85)))
    add("pickle_basket", "plastic_pp", cyl(38, (x, y, 55), (x, y, 102)).cut(cyl(36, (x, y, 57), (x, y, 103))).union(box(x - 3, x + 3, y - 2, y + 2, 102, 140)))

    # ------------------------------------------------------------------ hand-work end (left, outside arm reach)
    add("bench_pin", "wood_light", box(-830, -730, 380, 520, -22, -6).cut(box(-795, -765, 470, 521, -30, 0)))
    add("bench_pin_clamp", "steel_dark", box(-840, -720, 380, 420, -60, -40))
    add("bench_vise", "enamel_blue", cbox(-800, 300, 120, 90, 0, 60, 6).union(box(-850, -750, 345, 365, 60, 110)).union(box(-850, -750, 255, 275, 60, 110)))
    add("bench_vise_handle", "steel", cyl(5, (-800, 410, 85), (-800, 470, 85)).union(cyl(4, (-830, 470, 85), (-770, 470, 85))))
    add("cutting_mat", "mat_green", box(-880, -700, 40, 200, 0, 2))
    add("project_print", "part_orange", cbox(-780, 110, 70, 50, 2, 30, 6))

    # ------------------------------------------------------------------ hot cart (person only, parks at the left end)
    c = CART
    frame = box(c["x0"], c["x1"], c["y0"], c["y1"], c["top"] - 30, c["top"])  # top tray
    frame = frame.cut(box(c["x0"] + 8, c["x1"] - 8, c["y0"] + 8, c["y1"] - 8, c["top"] - 22, c["top"] + 1))
    frame = frame.union(box(c["x0"], c["x1"], c["y0"], c["y1"], c["shelf"] - 20, c["shelf"]))
    for x in (c["x0"] + 15, c["x1"] - 15):
        for y in (c["y0"] + 15, c["y1"] - 15):
            frame = frame.union(cbox(x, y, 25, 25, -680, 650, 2))
    add("cart_frame", "steel_dark", frame)
    add("cart_top_board", "cement_board", box(c["x0"] + 8, c["x1"] - 8, c["y0"] + 8, c["y1"] - 8, c["top"] - 22, c["top"] - 10))
    wheels = None
    for x in (c["x0"] + 40, c["x1"] - 40):
        for y in (c["y0"] + 40, c["y1"] - 40):
            w = cyl(40, (x - 12, y, -720), (x + 12, y, -720)).union(box(x - 18, x + 18, y - 14, y + 14, -690, -672))
            wheels = w if wheels is None else wheels.union(w)
    add("cart_wheels", "rubber", wheels)
    add("cart_handle", "steel", cyl(10, (c["x0"] - 40, c["y0"] + 60, c["top"] - 15), (c["x0"] - 40, c["y1"] - 60, c["top"] - 15))
        .union(box(c["x0"] - 45, c["x0"], c["y0"] + 55, c["y0"] + 65, c["top"] - 20, c["top"] - 10))
        .union(box(c["x0"] - 45, c["x0"], c["y1"] - 65, c["y1"] - 55, c["top"] - 20, c["top"] - 10)))
    # wall behind the cart: steel heat panel instead of pegboard
    add("heat_panel", "steel", box(c["x0"] - 60, c["x1"] + 40, WALL_Y, WALL_Y + 12, c["top"] - 20, 900))
    # kiln (small burnout / enamel kiln) on the cart top
    kx, ky, kz = c["x0"] + 165, 70.0, c["top"] - 10
    add("kiln_body", "enamel_grey", cbox(kx, ky, 260, 260, kz, 320, 14))
    add("kiln_door", "enamel_dark", box(kx - 95, kx + 95, ky + 130, ky + 140, kz + 50, kz + 250))
    add("kiln_handle", "steel", box(kx + 60, kx + 74, ky + 140, ky + 152, kz + 120, kz + 190))
    add("kiln_controller", "dark", box(kx - 80, kx + 10, ky + 130, ky + 142, kz + 260, kz + 310))
    add("kiln_display", "lcd_red", box(kx - 70, kx - 20, ky + 142, ky + 144, kz + 280, kz + 298))
    add("kiln_vent", "steel", cyl(16, (kx, ky, kz + 320), (kx, ky, kz + 350)))
    # fixed torch station: steel bracket, fire-brick turntable, gas solenoid, flame sensor, hood
    tx, ty, tz = c["x1"] - 150, 70.0, c["top"] - 10
    add("torch_base", "steel_dark", cbox(tx, ty, 260, 240, tz, 10, 6))
    add("torch_turntable", "steel", cyl(80, (tx - 30, ty + 10, tz + 10), (tx - 30, ty + 10, tz + 30)))
    for i in range(2):
        add(f"firebrick_{i}", "firebrick", cbox(tx - 30, ty - 25 + i * 70, 150, 64, tz + 30, 30, 1))
    stand = box(tx + 95, tx + 107, ty - 15, ty + 15, tz + 10, tz + 260)
    bracket = box(tx + 55, tx + 107, ty - 20, ty + 20, tz + 200, tz + 214).union(box(tx + 55, tx + 67, ty - 20, ty + 20, tz + 160, tz + 214))
    add("torch_stand", "steel", stand.union(bracket))
    add("torch_clamp", "steel_dark", cyl(22, (tx + 61, ty, tz + 170), (tx + 61, ty, tz + 205)).cut(cyl(16, (tx + 61, ty, tz + 160), (tx + 61, ty, tz + 210))))
    torch = cyl(15, (tx + 61, ty, tz + 130), (tx + 61, ty, tz + 240)).union(cone(15, 5, (tx + 61, ty, tz + 130), (tx + 15, ty, tz + 100))).union(cyl(5, (tx + 15, ty, tz + 100), (tx - 15, ty, tz + 80)))
    add("torch_body", "brass", torch)
    add("torch_flame", "flame", cone(4.5, 0.5, (tx - 15, ty, tz + 80), (tx - 40, ty, tz + 64)))
    add("flame_sensor", "steel", cyl(4, (tx - 100, ty + 60, tz + 30), (tx - 60, ty + 30, tz + 75)))
    add("gas_solenoid", "brass", cbox(tx + 101, ty + 50, 30, 30, tz + 60, 25))
    hood = box(tx - 150, tx + 150, ty - 120, ty + 140, tz + 460, tz + 466).union(cyl(45, (tx, ty, tz + 466), (tx, ty, tz + 560)))
    hood = hood.union(cyl(45, (tx, ty, tz + 560), (tx, WALL_Y + 12, tz + 560)))  # duct into the wall
    hood = hood.union(box(tx + 128, tx + 140, ty + 10, ty + 22, tz, tz + 460))  # mast
    add("fume_hood", "steel_dark", hood)
    # lower shelf: melting furnace, small vacuum caster, flasks, gas canister (stored, used on the cart top)
    sz = c["shelf"]
    add("furnace_body", "enamel_dark", fil(cyl(75, (c["x0"] + 110, 0, sz), (c["x0"] + 110, 0, sz + 240)), 8, ">Z"))
    add("caster_body", "enamel_grey", cbox(c["x0"] + 330, 30, 200, 200, sz, 160, 10))
    add("caster_well", "steel_dark", cyl(50, (c["x0"] + 330, 30, sz + 160), (c["x0"] + 330, 30, sz + 178)).cut(cyl(43, (c["x0"] + 330, 30, sz + 164), (c["x0"] + 330, 30, sz + 179))))
    add("gas_canister", "enamel_red", fil(cyl(35, (c["x1"] - 70, 180, sz), (c["x1"] - 70, 180, sz + 180)), 12, ">Z"))
    add("flasks", "steel", cyl(40, (c["x0"] + 330, 200, sz), (c["x0"] + 330, 200, sz + 90)).cut(cyl(38, (c["x0"] + 330, 200, sz + 2), (c["x0"] + 330, 200, sz + 91))))
    # extinguisher on a wall bracket between cart and bench
    ex, ey = -1700.0, WALL_Y + 70
    add("fire_extinguisher", "enamel_red", fil(cyl(50, (ex, ey, 150), (ex, ey, 520)), 20, ">Z").union(box(ex - 10, ex + 10, ey - 10, ey + 10, 520, 560)))
    add("extinguisher_bracket", "steel_dark", box(ex - 30, ex + 30, WALL_Y, ey - 45, 300, 330))
    return P


def _pegboard_tools(add):
    """Simplified hand tools on pegboard hooks (face at y = PEG_FACE, tools hang up to ~110 mm out)."""
    f = PEG_FACE

    def hook(x, z, out=60):
        return cyl(2.5, (x, f, z), (x, f + out, z)).union(cyl(2.5, (x, f + out, z), (x, f + out, z + 15)))

    hooks = None

    def h(x, z, out=60):
        nonlocal hooks
        k = hook(x, z, out)
        hooks = k if hooks is None else hooks.union(k)

    # --- left section: striking + cutting
    h(-820, 1050); add("tool_hammer", "wood_light", box(-830, -810, f + 45, f + 65, 760, 1050)); add("tool_hammer_head", "steel_dark", box(-860, -775, f + 40, f + 70, 1040, 1075))
    for i, x in enumerate((-740, -700, -660)):  # screwdrivers
        h(x, 1080, 30)
        add(f"tool_driver_{i}", ("handle_red", "handle_yellow", "handle_red")[i], cyl(12, (x, f + 35, 1000), (x, f + 35, 1090)))
        add(f"tool_driver_shaft_{i}", "steel", cyl(3, (x, f + 35, 860 + i * 20), (x, f + 35, 1000)))
    for i, x in enumerate((-590, -545, -500)):  # pliers (two handles + jaw)
        h(x, 1060)
        add(f"tool_pliers_{i}", "handle_blue", box(x - 14, x - 6, f + 50, f + 62, 900, 1010).union(box(x + 6, x + 14, f + 50, f + 62, 900, 1010)))
        add(f"tool_pliers_jaw_{i}", "steel", cone(9, 2, (x, f + 56, 1010), (x, f + 56, 1065)))
    h(-420, 1100, 40); add("tool_hacksaw", "steel_dark", box(-470, -460, f + 40, f + 50, 820, 1100).union(box(-470, -330, f + 40, f + 50, 1090, 1100)).union(box(-340, -330, f + 40, f + 50, 820, 1100)))
    add("tool_hacksaw_blade", "steel", box(-462, -338, f + 43, f + 47, 826, 830))
    h(-280, 1100, 40); add("tool_jewelers_saw", "steel", box(-300, -295, f + 42, f + 46, 900, 1100).union(box(-300, -200, f + 42, f + 46, 1095, 1100)).union(box(-205, -200, f + 42, f + 46, 900, 1100)))
    for i, x in enumerate((-160, -130, -100, -70)):  # files
        h(x, 1080, 25)
        add(f"tool_file_{i}", "steel_dark", box(x - 5, x + 5, f + 25, f + 30, 880, 1060))
    # --- centre: measuring + parts bins + shelf
    h(-10, 1090, 30); add("tool_steel_rule", "steel", box(-20, 0, f + 25, f + 28, 790, 1090))
    h(40, 1080); add("tool_calipers", "steel", box(32, 40, f + 50, f + 56, 860, 1080).union(box(40, 75, f + 50, f + 56, 1060, 1070)).union(box(40, 75, f + 50, f + 56, 1030, 1040)))
    h(120, 1060); add("tool_square", "steel", box(100, 108, f + 50, f + 56, 860, 1060).union(box(100, 260, f + 50, f + 56, 860, 868)))
    add("tool_square_stock", "steel_dark", box(96, 112, f + 46, f + 60, 960, 1060))
    h(330, 1080); add("tool_tape", "handle_yellow", cbox(330, f + 55, 70, 40, 1000, 75, 12))
    add("parts_rail", "steel_dark", box(-150, 450, f, f + 12, 800, 812))
    for i in range(6):
        x = -120 + i * 95
        add(f"parts_bin_{i}", ("bin_blue", "bin_red", "bin_yellow", "bin_blue", "bin_red", "bin_yellow")[i], box(x - 42, x + 42, f + 4, f + 110, 700, 790).cut(box(x - 38, x + 38, f + 8, f + 106, 715, 800)).cut(box(x - 43, x + 43, f + 70, f + 111, 755, 800)))
    add("shelf", "wood_light", box(-150, 450, f, f + 160, 480, 498).union(box(-140, -130, f, f + 140, 420, 480)).union(box(430, 440, f, f + 140, 420, 480)))
    for i, (x, r, hgt, m) in enumerate(((-100, 22, 120, "bottle_white"), (-50, 18, 90, "bottle_amber"), (0, 25, 140, "bottle_white"), (60, 20, 100, "bottle_blue"), (120, 30, 70, "bottle_amber"))):
        add(f"shelf_bottle_{i}", m, fil(cyl(r, (x, f + 80, 498), (x, f + 80, 498 + hgt)), r * 0.4, ">Z"))
    add("shelf_filament", "filament_orange", cyl(100, (360, f + 40, 596), (360, f + 105, 596)).cut(cyl(28, (360, f + 39, 596), (360, f + 106, 596))))
    add("shelf_filament_rod", "steel", cyl(12, (360, f, 596), (360, f + 120, 596)))
    # --- right section: clamps, drill, safety gear, wire spools
    for i, x in enumerate((520, 580)):  # F-clamps
        h(x, 1100, 40)
        add(f"tool_fclamp_{i}", "steel", box(x - 5, x + 5, f + 40, f + 50, 820, 1100).union(box(x - 5, x + 60, f + 40, f + 50, 1080, 1100)))
        add(f"tool_fclamp_jaw_{i}", "handle_red", box(x - 5, x + 60, f + 38, f + 52, 860, 900).union(cyl(10, (x + 50, f + 45, 830), (x + 50, f + 45, 860))))
    add("drill_holster", "steel_dark", box(640, 760, f, f + 90, 820, 835).cut(cyl(22, (700, f + 50, 810), (700, f + 50, 840))))
    add("tool_drill", "handle_green", cyl(20, (700, f + 50, 840), (700, f + 50, 1000)).union(cbox(700, f + 50, 50, 60, 1000, 70, 10)).union(cbox(700, f + 50, 60, 70, 760, 60, 8)))
    add("tool_drill_chuck", "steel_dark", cyl(14, (700, f + 50, 790), (700, f + 50, 820)))
    h(820, 1050); add("safety_glasses", "glass_tint", box(785, 855, f + 55, f + 62, 1000, 1030))
    h(820, 960); add("ear_muffs", "handle_red", cyl(30, (790, f + 55, 920), (790, f + 75, 920)).union(cyl(30, (850, f + 55, 920), (850, f + 75, 920))).union(box(788, 852, f + 62, f + 68, 950, 965)))
    for i, x in enumerate((560, 640, 720)):  # solder / wire spools on a rod
        add(f"spool_{i}", ("copper", "silver", "steel")[i], cyl(28, (x - 25, f + 70, 640), (x + 25, f + 70, 640)).cut(cyl(9, (x - 26, f + 70, 640), (x + 26, f + 70, 640))))
    add("spool_rod", "steel", cyl(6, (510, f + 70, 640), (770, f + 70, 640)).union(box(505, 515, f, f + 75, 630, 650)).union(box(765, 775, f, f + 75, 630, 650)))
    add("paper_towel", "paper", cyl(55, (-860, f + 70, 560), (-640, f + 70, 560)))
    add("paper_towel_holder", "steel_dark", box(-875, -860, f, f + 75, 540, 580).union(box(-640, -625, f, f + 75, 540, 580)))
    add("pegboard_hooks", "steel", hooks)
    add("led_bar", "led", box(-880, 880, WALL_Y + 40, WALL_Y + 70, 1230, 1245))


def reach():
    sys.path.insert(0, str(REPO / "gantry-arm" / "software" / "lerobot_robot_gantry_arm"))
    from lerobot_robot_gantry_arm.kinematics import IKError, Kinematics, Pose

    k = Kinematics()
    res = {}
    for name, s in STATIONS.items():
        x, y = s["xy"]
        try:
            q = k.ik(Pose(x, y, s["z"]))
            res[name] = dict(ok=True, gantry_x=round(q["gantry_x"]), gantry_z=round(q["gantry_z"]))
        except IKError:
            res[name] = dict(ok=False)
    return res


def main():
    import cadquery as cq

    OUT.mkdir(parents=True, exist_ok=True)
    man = []
    for name, role, shape in build():
        cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.1, angularTolerance=0.2)
        man.append(dict(name=name, role=role, valid=shape.val().isValid()))
    (OUT / "manifest.json").write_text(json.dumps(man, indent=1))
    (OUT / "layout.json").write_text(json.dumps(dict(bench=BENCH, peg=PEG, cart=CART, stations=STATIONS, wall_y=WALL_Y, peg_face=PEG_FACE), indent=1))
    r = reach()
    (OUT / "reach.json").write_text(json.dumps(r, indent=1))
    print(len(man), "parts; invalid:", [m["name"] for m in man if not m["valid"]])
    for n, v in r.items():
        print(f"  {n:11s} {'REACH' if v['ok'] else 'NO REACH'} {v}")


if __name__ == "__main__":
    main()
