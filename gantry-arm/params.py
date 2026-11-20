"""Single source of truth for every dimension, mass and rating in the Gantry Arm.

Units: millimetres, grams, degrees, N·m (torque ratings) unless the name says otherwise.
Both the simulation (sim/) and the CAD (cad/) import this file. Change a number here,
re-run `python sim/build_model.py` and `python cad/build_all.py`.

Frame convention (world):
    +X  along the X rail
    +Y  outward from the Z column toward the work area
    +Z  up
"""

from dataclasses import dataclass, field

# ---------------------------------------------------------------- printer
BED = (256.0, 256.0, 256.0)  # Bambu P2S build volume, mm
BED_MARGIN = 6.0  # keep parts this far from the bed edge

# ---------------------------------------------------------------- materials
PETG_DENSITY = 1.27e-3  # g/mm^3
PETG_E_PRINTED = 1500.0  # MPa, conservative for 4 walls + 40% gyroid (bulk ~2000)
AL_E = 69000.0  # MPa, 6063-T5 extrusion
AL_6061_E = 69000.0


# ---------------------------------------------------------------- servos
@dataclass(frozen=True)
class Servo:
    model: str
    stall_Nm: float  # at 12 V
    rated_Nm: float  # manufacturer "rated" (continuous) torque at 12 V
    mass_g: float
    body: tuple  # L x W x H mm (H along output shaft)
    stall_A: float


KGCM = 0.0980665  # N·m per kg·cm
STS3215 = Servo("sts3215", 30 * KGCM, 10 * KGCM, 55.0, (45.2, 24.7, 35.0), 2.7)
STS3250 = Servo("sts3250", 50 * KGCM, 16 * KGCM, 74.5, (45.2, 24.7, 35.0), 4.2)
# J5 roll micro servo (owner choice 2026-11-20). 6 V part: fed by a small buck in the wrist; same TTL bus.
# Vendors disagree on its size (23.2x12.1x28.5 vs 32x12x27.5): designed for the first, VERIFY on the real part.
STS3032 = Servo("sts3032", 4.5 * KGCM, 1.5 * KGCM, 20.0, (23.2, 12.1, 28.5), 1.2)
SERVO_SHAFT_FROM_END = 11.5  # STS3215/3250 output shaft distance from the near short end (verify on the drawing)

UTILISATION_FLAG = 0.40  # flag any joint above 40 % of stall (user rule)

# ---------------------------------------------------------------- gantry X
X_PROFILE = (20.0, 60.0)  # 2060 V-slot, lying flat on the table
X_RAIL_LEN = 1000.0
X_TRAVEL = 800.0
X_CARRIAGE = dict(length=140.0, width=100.0, thick=10.0, wheel_pitch=110.0, wheels=6)
X_PULLEY_TEETH = 20  # GT2 -> 40 mm / rev
X_ACCEL = 1000.0  # mm/s^2 design acceleration
X_SPEED = 200.0  # mm/s

# ---------------------------------------------------------------- gantry Z
Z_PROFILE = (20.0, 40.0)  # 2040 V-slot, 40 mm side faces +Y (stiff against platform moment)
Z_COLUMN_LEN = 700.0
Z_TRAVEL = 500.0
Z_CARRIAGE = dict(length=120.0, width=70.0, thick=8.0, row_pitch=90.0, wheels_per_row=4)
Z_SCREW = dict(dia=8.0, lead=2.0, starts=1, mu_nut=0.12)  # T8x2 single start: self-locking
Z_BOTTOM_CLEAR = 200.0  # column base to lowest platform top (clears the X carriage shroud)
Z_SPEED = 20.0  # mm/s (T8x2 -> 600 rpm)

# lift platform: cantilever from Z carriage face to the J1 axis
PLATFORM = dict(offset=95.0, width=90.0, plate=8.0, rib_h=36.0, rib_t=6.0, ribs=2, mass_g=210.0)

# ---------------------------------------------------------------- arm
ARM = dict(
    base_h=120.0,  # platform top -> J2 axis (shoulder puck sits on the J1 column)
    upper=165.0,  # J2 -> J3
    fore=150.0,  # J3 -> J4
    wrist=55.0,  # J4 -> J5 output flange (roll housing holds the J5 servo)
    coupler=16.0,  # coupler stack height (both halves, 8 + 8)
    tool_tcp=64.0,  # coupler tool face -> gripper TCP
)

# ---- design variants (env vars so a preview can be built/rendered without editing this file)
import os as _os

# Forearm extension, mm. Owner chose 0 (450 mm reach, 500 g). The env var only builds previews.
FORE_EXT = float(_os.environ.get("GANTRY_FORE_EXT", "0"))
ARM["fore"] += FORE_EXT
REACH = ARM["upper"] + ARM["fore"] + ARM["wrist"] + ARM["coupler"] + ARM["tool_tcp"]
# Stereo-camera "eyes" head on the column top (owner chose it; replaces the plain overhead camera)
HEAD = _os.environ.get("GANTRY_HEAD", "1") == "1"
HEAD_GEOM = dict(width=112.0, depth=48.0, height=46.0, eye_dx=30.0, lens_r=8.5, ring_r=(11.0, 15.0), tilt_deg=38.0, center=(0.0, 18.0, 792.0))

# Elbow: 2 = ganged twin STS3250 (confirmed by owner 2026-11-19, see SIM_REPORT), 1 = single
ELBOW_SERVOS = 2

# ---------------------------------------------------------------- industrial design language
STYLE = dict(
    wall=2.4,  # shell wall (6 perimeters of 0.4)
    link_thick=26.0,  # offset side-link thickness along the joint axis
    fillet=7.0,  # outer edge radius on links
    gap=2.0,  # running gap between a puck face and the next link
    shoulder_puck=(48.0, 96.0),  # radius, width
    elbow_puck_r=44.0,  # 40 -> 44: room for the twin-servo gear pair (see JOINT_GEAR)
    wrist_puck=(32.0, 64.0),
    roll_r=30.0,  # 28 -> 30: room for the off-axis J5 micro servo
    link_r=dict(upper=(46.0, 40.0), fore=(38.0, 32.0)),  # hub radii at each end
    led=dict(width=5.0, depth=1.6, z0=30.0, z1=130.0),
)
REACH = ARM["upper"] + ARM["fore"] + ARM["wrist"] + ARM["coupler"] + ARM["tool_tcp"]  # J2->TCP

# printed link masses (from CAD estimate, PETG 4 walls / 40 % infill)
MASS = dict(
    turret=120.0,  # J1 turret incl. bearing, carries both J2 servos
    upper=115.0,
    fore=95.0,
    wrist=35.0,
    coupler_robot=32.0,
    coupler_tool=26.0,
    gripper_parts=60.0,  # fingers, rack, housing (servo counted separately)
    payload=500.0,
    payload_offset=15.0,  # payload CoM off the J5 roll axis (realistic grasp error)
)

# joint servos
JOINT_SERVOS = {
    "base_yaw": (STS3215, 1),
    "shoulder_lift": (STS3250, 2),
    "elbow_flex": (STS3250, ELBOW_SERVOS),
    "wrist_flex": (STS3215, 1),
    "wrist_roll": (STS3032, 1),  # off-axis, 1:1 m0.5 36T/36T gear pair to the roll shaft
    "gripper": (STS3215, 1),
}
# twin-servo geared pairs (owner choice 2026-11-20): both servos of a joint drive one steel hub gear
# through their own pinion. Ratio 2:1 -> the servos turn > 1 rev over the joint range, so they run in
# Feetech multi-turn mode and an AS5600 magnetic encoder on the hub gives the true joint angle.
JOINT_GEAR = {  # joint: (ratio, mesh efficiency)
    "shoulder_lift": (2.0, 0.95),
    "elbow_flex": (2.0, 0.95),
}
GEARS = {  # steel spur gears, module 1, 20 deg pressure angle (stock parts; pinion bolted to the servo horn)
    # two DIFFERENT pinions per pair (vernier): servo A turns 2.000x the joint, servo B z_hub/z_pinion_b x.
    # Their two absolute single-turn encoders then give the absolute joint angle at power-up
    # (no extra sensor; the axis stays free for the hollow wiring shaft). See docs/MECHANICS.md.
    "shoulder_lift": dict(module=1.0, z_pinion=(18, 17), z_hub=36, face=10.0),  # centre distances 27.0 / 26.5 mm
    "elbow_flex": dict(module=1.0, z_pinion=(15, 14), z_hub=30, face=10.0),  # 22.5 / 22.0 mm; 14T/15T undercut: profile shift +0.2 or accept
    "wrist_roll": dict(module=0.5, z_pinion=(36,), z_hub=36, face=5.0),  # 1:1 offset drive, centre distance 18 mm
}
PRELOAD_NM = 0.15  # anti-backlash: the two servos of a pair push against each other with this torque (joint side)
# absolute joint angle on the geared pairs comes from the vernier pinions (owner asked for joint encoders;
# an on-axis AS5600 cannot share the axis with the hollow wiring shaft, so the vernier does the same job)
JOINT_ENCODERS = ("shoulder_lift", "elbow_flex")
BEARINGS = {  # joint: list of (designation, bore, OD, width) in mm
    "base_yaw": [("61812-2RS", 60, 78, 10), ("61812-2RS", 60, 78, 10)],  # on a fixed spigot, 26 mm apart: carry the arm moment; Oldham coupling -> servo sees torque only
    "shoulder_lift": [("6804-2RS", 20, 32, 7), ("6804-2RS", 20, 32, 7)],  # hollow shaft 20/15 (aluminium tube)
    "elbow_flex": [("6803-2RS", 17, 26, 5), ("6803-2RS", 17, 26, 5)],  # hollow shaft 17/13
    "wrist_flex": [("6806-2RS", 30, 42, 7)],  # around the J4 horn in the -Y wall: carries the wrist moment
    "wrist_roll": [("6805-2RS", 25, 37, 7)],  # roll shaft at the flange; carries the tool moment, servo sees torque only
}


def joint_capacity(joint: str) -> tuple[float, float, int]:
    """(stall N·m, rated N·m, servo count) at the joint, incl. gear ratio and mesh efficiency."""
    s, n = JOINT_SERVOS[joint]
    ratio, eff = JOINT_GEAR.get(joint, (1.0, 1.0))
    return s.stall_Nm * n * ratio * eff, s.rated_Nm * n * ratio * eff, n


def joint_kp_scale(joint: str) -> float:
    """Joint-side position stiffness relative to ONE direct-drive servo: count x ratio^2."""
    _, n = JOINT_SERVOS[joint]
    ratio, _ = JOINT_GEAR.get(joint, (1.0, 1.0))
    return n * ratio * ratio


DUAL_SHARE = 0.85  # worst-case load split for the twin shoulder servos (one carries 1/(2*0.85))

JOINT_LIMITS = {  # degrees
    "base_yaw": (-150, 150),
    "shoulder_lift": (-100, 135),  # forward to 135: reach the table past the platform (checked in IK collision model)
    "elbow_flex": (-140, 140),
    "wrist_flex": (-110, 110),
    "wrist_roll": (-180, 180),
}
GRIPPER = dict(stroke=40.0, pinion_r=10.0, mu_finger=0.5, safety=2.0)

# shoulder counterbalance: ideal zero-free-length spring (spring + cable over a pulley)
# anchor `a` above J2 on the turret, attach `b` up the upper arm.
SPRING = dict(a=40.0, b=60.0, k=None)  # k (N/mm) chosen by sim/static_check.py if None

# ---------------------------------------------------------------- V-wheels
# Delrin V-wheel effective radial stiffness incl. eccentric spacer. Large uncertainty:
# report results for the low and high value.
WHEEL_K_N_PER_MM = (300.0, 1500.0)

# 2040 / 2060 V-slot second moments of area (OpenBuilds table), mm^4
I_2040 = dict(strong=48163.0, weak=12305.0)
I_2060 = dict(strong=149336.0, weak=17602.0)


@dataclass
class Cameras:
    wrist: str = "USB UVC 640x480 MJPG"
    overhead: str = "USB UVC 1280x720 MJPG"


CAMERAS = Cameras()


# hardware not modelled in CAD (screws, inserts, wiring, bearings, LED strip, PCBs), grams per body
# z_carriage: 6 mm Al carriage plate + 8 V-wheels + eccentric spacers + T8 anti-backlash nut
HARDWARE_G = dict(z_carriage=260.0, turret=80.0, upper_arm=25.0, forearm=25.0, wrist=15.0, tool=20.0)

# densities for CAD mass properties, g/mm^3
DENSITY = dict(PETG=1.27e-3, steel=7.9e-3, brass=8.5e-3, rubber=1.2e-3, magnet=7.5e-3, aluminium=2.7e-3, glass=2.5e-3)


# Tool rack (workcell, on the table): passive tool changer.
# Each tool's coupler half has a parking groove; the rack fork slides into it horizontally (+Y),
# and a cam channel beside the fork holds the bayonet sleeve's lever open while docked.
RACK = dict(
    slots={1: 250.0, 2: 315.0, 3: 375.0, 4: 450.0},  # spaced for the 96 mm phone mount; tool id -> world x of the slot centre (1 gripper, 2 suction, 3 pen, 4 phone)
    y=275.0,  # world y of the slot centre (fork opening faces -Y, toward the robot)
    tcp_z=110.0,  # gripper-TCP-equivalent height while docked (raised so 150 mm spindle tools clear the table)
    riser=80.0,  # rack modules stand on this riser block
    insert_mm=45.0,  # horizontal travel into the fork
    lift_mm=20.0,  # Z-gantry lift that breaks the magnets
    groove=(10.0, 14.0, 2.6),  # groove on the tool coupler half: tool-frame z from..to, depth (mm)
)
