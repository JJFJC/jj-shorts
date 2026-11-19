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
Z_BOTTOM_CLEAR = 120.0  # column base to lowest platform top
Z_SPEED = 20.0  # mm/s (T8x2 -> 600 rpm)

# lift platform: cantilever from Z carriage face to the J1 axis
PLATFORM = dict(offset=95.0, width=90.0, plate=8.0, rib_h=36.0, rib_t=6.0, ribs=2, mass_g=210.0)

# ---------------------------------------------------------------- arm
ARM = dict(
    base_h=70.0,  # platform top -> J2 axis
    upper=175.0,  # J2 -> J3
    fore=160.0,  # J3 -> J4
    wrist=40.0,  # J4 -> coupler face (includes J5 roll servo)
    coupler=18.0,  # coupler stack height (both halves)
    tool_tcp=57.0,  # coupler tool face -> gripper TCP
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
    "elbow_flex": (STS3250, 1),
    "wrist_flex": (STS3215, 1),
    "wrist_roll": (STS3215, 1),
    "gripper": (STS3215, 1),
}
DUAL_SHARE = 0.85  # worst-case load split for the twin shoulder servos (one carries 1/(2*0.85))

JOINT_LIMITS = {  # degrees
    "base_yaw": (-150, 150),
    "shoulder_lift": (-100, 100),
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
