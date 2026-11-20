"""Parametric kinematic/mass model of the Gantry Arm -> URDF and MJCF.

All geometry comes from params.py. Masses are lumped as boxes per component so the
same numbers feed MuJoCo (geom masses) and URDF (combined inertials).

Zero pose: gantry at home, arm pointing straight up, gripper open.
Positive shoulder/elbow/wrist pitch tilts the arm toward +Y (out over the work area).
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import params as P  # noqa: E402

MM = 1e-3
G = 1e-3  # g -> kg


@dataclass
class Part:
    """A lumped component: box of `size` (mm) and mass (g) centred at `pos` (mm, body frame)."""

    name: str
    mass_g: float
    pos: tuple
    size: tuple
    rgba: str = "0.8 0.5 0.1 1"


@dataclass
class Body:
    name: str
    parent: str | None
    pos: tuple  # joint origin in parent frame, mm
    joint: str | None = None
    jtype: str | None = None  # hinge | slide
    axis: tuple = (0, 0, 1)
    limit: tuple | None = None  # (lo, hi) deg for hinge, mm for slide
    quat: tuple = (1, 0, 0, 0)
    parts: list[Part] = field(default_factory=list)
    sites: dict = field(default_factory=dict)

    def mass(self) -> float:
        return sum(p.mass_g for p in self.parts)

    def com(self) -> tuple:
        m = self.mass()
        if m == 0:
            return (0.0, 0.0, 0.0)
        return tuple(sum(p.mass_g * p.pos[i] for p in self.parts) / m for i in range(3))

    def inertia(self) -> tuple:
        """Diagonal inertia (kg·m²) about the combined CoM (box + parallel axis, off-diagonals dropped)."""
        c = self.com()
        ixx = iyy = izz = 0.0
        for p in self.parts:
            m = p.mass_g * G
            sx, sy, sz = (s * MM for s in p.size)
            dx, dy, dz = ((p.pos[i] - c[i]) * MM for i in range(3))
            ixx += m * (sy**2 + sz**2) / 12 + m * (dy**2 + dz**2)
            iyy += m * (sx**2 + sz**2) / 12 + m * (dx**2 + dz**2)
            izz += m * (sx**2 + sy**2) / 12 + m * (dx**2 + dy**2)
        return (max(ixx, 1e-7), max(iyy, 1e-7), max(izz, 1e-7))


def column_mass_g() -> float:
    # 2040 V-slot ~ 0.73 kg/m (OpenBuilds), 2060 ~ 1.06 kg/m
    return 0.73 * P.Z_COLUMN_LEN


def build_chain() -> list[Body]:
    A, M = P.ARM, P.MASS
    s50, s15 = P.STS3250, P.STS3215
    zh = P.Z_PROFILE[1]
    pw = P.PLATFORM

    bodies: list[Body] = []
    # ---- X carriage (prismatic X). Rail sits on the table at z=0, carriage on top of it.
    x0 = -P.X_TRAVEL / 2
    xc = Body(
        "x_carriage", None, (x0, 0, P.X_PROFILE[0]), "gantry_x", "slide", (1, 0, 0), (0, P.X_TRAVEL),
        parts=[
            Part("x_plate", 350.0, (0, 0, 5), (P.X_CARRIAGE["length"], P.X_CARRIAGE["width"], 10), "0.3 0.3 0.35 1"),
            Part("x_motor", 280.0, (0, -60, 25), (42, 42, 40), "0.1 0.1 0.1 1"),
            Part("column", column_mass_g(), (0, 0, 10 + P.Z_COLUMN_LEN / 2), (P.Z_PROFILE[0], zh, P.Z_COLUMN_LEN), "0.75 0.75 0.78 1"),
            Part("z_motor", 280.0, (0, 0, 10 + P.Z_COLUMN_LEN + 25), (42, 42, 48), "0.1 0.1 0.1 1"),
        ],
    )
    bodies.append(xc)
    # ---- Z carriage + platform (prismatic Z). Carriage rides on the +Y face of the column.
    z0 = 10 + P.Z_BOTTOM_CLEAR
    zc = Body(
        "z_carriage", "x_carriage", (0, zh / 2 + 12, z0), "gantry_z", "slide", (0, 0, 1), (0, P.Z_TRAVEL),
        parts=[
            Part("z_plate", 160.0, (0, 0, -P.Z_CARRIAGE["length"] / 2), (P.Z_CARRIAGE["width"], 8, P.Z_CARRIAGE["length"]), "0.3 0.3 0.35 1"),
            Part("platform", pw["mass_g"], (0, pw["offset"] / 2, -pw["plate"] / 2), (pw["width"], pw["offset"] + 45, pw["plate"]), "0.85 0.45 0.1 1"),
        ],
        sites={"platform_top": (0, pw["offset"], 0)},
    )
    bodies.append(zc)
    # ---- J1 base yaw. Rotate so the arm's local +x points world +Y.
    turret = Body(
        "turret", "z_carriage", (0, pw["offset"], 0), "base_yaw", "hinge", (0, 0, 1), P.JOINT_LIMITS["base_yaw"],
        quat=(math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)),
        parts=[
            Part("turret", M["turret"], (0, 0, A["base_h"] * 0.45), (70, 70, A["base_h"])),
            Part("j1_servo", s15.mass_g, (0, 0, -20), (45, 25, 35), "0.1 0.1 0.1 1"),
            Part("j2_servo_a", s50.mass_g, (0, 32, A["base_h"]), (25, 35, 45), "0.1 0.1 0.1 1"),
            Part("j2_servo_b", s50.mass_g, (0, -32, A["base_h"]), (25, 35, 45), "0.1 0.1 0.1 1"),
        ],
        sites={"spring_anchor": (0, 0, A["base_h"] + P.SPRING["a"])},
    )
    bodies.append(turret)
    upper = Body(
        "upper_arm", "turret", (0, 0, A["base_h"]), "shoulder_lift", "hinge", (0, 1, 0), P.JOINT_LIMITS["shoulder_lift"],
        parts=[
            Part("upper_link", M["upper"], (0, 0, A["upper"] / 2), (30, 40, A["upper"])),
        ],
        sites={"spring_attach": (0, 0, P.SPRING["b"])},
    )
    bodies.append(upper)
    fore = Body(
        "forearm", "upper_arm", (0, 0, A["upper"]), "elbow_flex", "hinge", (0, 1, 0), P.JOINT_LIMITS["elbow_flex"],
        parts=[
            Part("j3_servo", s50.mass_g * P.ELBOW_SERVOS, (0, 0, 0), (25, 35 * P.ELBOW_SERVOS, 45), "0.1 0.1 0.1 1"),
            Part("fore_link", M["fore"], (0, 0, A["fore"] / 2), (28, 36, A["fore"])),
        ],
    )
    bodies.append(fore)
    wrist = Body(
        "wrist", "forearm", (0, 0, A["fore"]), "wrist_flex", "hinge", (0, 1, 0), P.JOINT_LIMITS["wrist_flex"],
        parts=[
            Part("j4_servo", s15.mass_g, (0, 0, 0), (25, 35, 45), "0.1 0.1 0.1 1"),
            Part("wrist_link", M["wrist"], (0, 0, A["wrist"] * 0.5), (30, 30, A["wrist"])),
            Part("j5_servo", s15.mass_g, (0, 0, A["wrist"] * 0.45), (25, 45, 35), "0.1 0.1 0.1 1"),
        ],
    )
    bodies.append(wrist)
    c, t = A["coupler"], A["tool_tcp"]
    roll = Body(
        "tool", "wrist", (0, wrist_center_y(), A["wrist"]), "wrist_roll", "hinge", (0, 0, 1), P.JOINT_LIMITS["wrist_roll"],
        parts=[
            Part("coupler_robot", M["coupler_robot"], (0, 0, c * 0.25), (50, 50, c / 2), "0.2 0.5 0.9 1"),
            Part("coupler_tool", M["coupler_tool"], (0, 0, c * 0.75), (50, 50, c / 2), "0.2 0.7 0.4 1"),
            Part("gripper_servo", s15.mass_g, (0, 0, c + 22), (45, 25, 35), "0.1 0.1 0.1 1"),
            Part("gripper_body", M["gripper_parts"], (0, 0, c + t * 0.55), (60, 20, t * 0.6)),
        ],
        sites={"tcp": (0, 0, c + t)},
    )
    bodies.append(roll)
    # payload lives in its own fixed body so its mass can be switched on/off at runtime
    bodies.append(
        Body("payload", "tool", (0, M["payload_offset"], c + t), parts=[Part("payload", M["payload"], (0, 0, 0), (40, 40, 40), "0.9 0.2 0.2 0.6")])
    )
    for side, sgn in (("left", 1), ("right", -1)):
        bodies.append(
            Body(
                f"finger_{side}", "tool", (0, 0, c + t - 10), f"gripper_{side}", "slide", (sgn, 0, 0), (0, P.GRIPPER["stroke"] / 2),
                parts=[Part(f"finger_{side}", 0.5 if CAD_PROPS else 8.0, (sgn * 4, 0, 5), (6, 18, 30), "0.6 0.6 0.6 1")],
            )
        )
    if CAD_PROPS:
        apply_cad_props(bodies)
    return bodies


# --------------------------------------------------------------------------- CAD mass properties

CAD_PROPS_FILE = ROOT / "cad" / "out" / "mass_props.json"
CAD_PROPS = json.loads(CAD_PROPS_FILE.read_text()) if CAD_PROPS_FILE.exists() else None
# lumped estimates that the CAD shells replace, per body
REPLACED = {
    "z_carriage": {"z_plate", "platform"},
    "turret": {"turret"},
    "upper_arm": {"upper_link"},
    "forearm": {"fore_link"},
    "wrist": {"wrist_link"},
    "tool": {"coupler_robot", "coupler_tool", "gripper_body"},
}


def wrist_center_y() -> float:
    """Lateral (joint-axis) position of the J5 roll axis; mirrors cad/parts.py."""
    return 48.0 - (82.0 if P.ELBOW_SERVOS == 2 else 48.0) + 32.0


def apply_cad_props(bodies):
    """Swap lumped link estimates for CAD-measured printed shells + hardware (mass, CoM)."""
    for b in bodies:
        if b.name not in REPLACED or b.name not in CAD_PROPS:
            continue
        cp = CAD_PROPS[b.name]
        b.parts = [p for p in b.parts if p.name not in REPLACED[b.name]]
        b.parts.append(Part(f"cad_{b.name}", cp["mass_g"], tuple(cp["com"]), tuple(max(5.0, x) for x in cp["size"]), "0.85 0.85 0.85 1"))


# --------------------------------------------------------------------------- MJCF


def _f(v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def to_mjcf(spring_k_N_per_mm: float = 0.0) -> str:
    bodies = build_chain()
    kids: dict[str | None, list[Body]] = {}
    for b in bodies:
        kids.setdefault(b.parent, []).append(b)

    def emit(b: Body, ind: int) -> str:
        sp = "  " * ind
        out = [f'{sp}<body name="{b.name}" pos="{_f(x * MM for x in b.pos)}" quat="{_f(b.quat)}">']
        if b.joint:
            if b.jtype == "hinge":
                lo, hi = (math.radians(x) for x in b.limit)
                cls = ""
                out.append(f'{sp}  <joint name="{b.joint}" type="hinge" axis="{_f(b.axis)}" range="{lo:.4f} {hi:.4f}"{cls}/>')
            else:
                lo, hi = (x * MM for x in b.limit)
                cls = ' class="gantry"' if b.joint.startswith("gantry") else ""
                out.append(f'{sp}  <joint name="{b.joint}" type="slide" axis="{_f(b.axis)}" range="{lo:.4f} {hi:.4f}"{cls}/>')
        for p in b.parts:
            out.append(
                f'{sp}  <geom name="{p.name}" type="box" size="{_f(s * MM / 2 for s in p.size)}" '
                f'pos="{_f(x * MM for x in p.pos)}" mass="{p.mass_g * G:.6g}" rgba="{p.rgba}"/>'
            )
        for sname, spos in b.sites.items():
            out.append(f'{sp}  <site name="{sname}" pos="{_f(x * MM for x in spos)}" size="0.006"/>')
        for k in kids.get(b.name, []):
            out.append(emit(k, ind + 1))
        out.append(f"{sp}</body>")
        return "\n".join(out)

    rail = P.X_PROFILE
    tree = "\n".join(emit(b, 2) for b in kids[None])
    act = []
    for name, (servo, n) in P.JOINT_SERVOS.items():
        if name == "gripper":
            continue
        f = P.joint_capacity(name)[0]
        kp = 17.8 * P.joint_kp_scale(name) * servo.stall_Nm / P.STS3215.stall_Nm
        act.append(f'    <position name="{name}" joint="{name}" kp="{kp:.3g}" forcerange="{-f:.3f} {f:.3f}"/>')
    # gripper: rack and pinion -> finger force = torque / pinion radius
    fg = P.STS3215.stall_Nm / (P.GRIPPER["pinion_r"] * MM)
    act.append(f'    <position name="gripper" joint="gripper_left" kp="400" forcerange="{-fg:.1f} {fg:.1f}"/>')
    # gantry: NEMA17 via GT2 (X) and T8x2 (Z) -> far stronger than needed, modelled as stiff position servos
    act.append('    <position name="gantry_x" joint="gantry_x" kp="20000" forcerange="-60 60"/>')
    act.append('    <position name="gantry_z" joint="gantry_z" kp="40000" forcerange="-400 400"/>')
    k_si = spring_k_N_per_mm * 1000.0
    return f"""<mujoco model="gantry_arm">
  <!-- generated by sim/model.py from params.py; do not edit by hand -->
  <compiler angle="radian" inertiafromgeom="true"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <default>
    <geom contype="0" conaffinity="0"/>
    <!-- STS3215 class measured by TheRobotStudio SO-101 (Apache-2.0), applied to all bus servos -->
    <joint damping="0.60" frictionloss="0.052" armature="0.028"/>
    <default class="gantry"><joint damping="5"/></default>
  </default>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <worldbody>
    <light pos="0.5 -1 1.5" dir="-0.3 0.6 -1"/>
    <geom name="table" type="plane" size="0.8 0.6 0.01" rgba="0.92 0.92 0.9 1"/>
    <geom name="x_rail" type="box" size="{P.X_RAIL_LEN * MM / 2} {rail[1] * MM / 2} {rail[0] * MM / 2}" pos="0 0 {rail[0] * MM / 2}" rgba="0.75 0.75 0.78 1"/>
{tree}
  </worldbody>
  <equality>
    <joint joint1="gripper_right" joint2="gripper_left" polycoef="0 1 0 0 0"/>
  </equality>
  <tendon>
    <!-- shoulder counterbalance: zero-free-length spring (extension spring + cable over pulley) -->
    <spatial name="shoulder_spring" stiffness="{k_si:.6g}" springlength="0 0" width="0.002" rgba="0.9 0.9 0.1 1">
      <site site="spring_anchor"/>
      <site site="spring_attach"/>
    </spatial>
  </tendon>
  <actuator>
{chr(10).join(act)}
  </actuator>
</mujoco>
"""


# --------------------------------------------------------------------------- URDF


def _rpy_from_quat(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def to_urdf() -> str:
    """URDF with the same chain. The shoulder spring cannot be expressed in URDF (see SIM_REPORT)."""
    bodies = build_chain()
    out = ['<?xml version="1.0"?>', '<robot name="gantry_arm">', "  <!-- generated by sim/model.py from params.py -->"]
    rail = P.X_PROFILE
    out.append(
        f'  <link name="world_base"><visual><origin xyz="0 0 {rail[0] * MM / 2}"/><geometry><box size="{P.X_RAIL_LEN * MM} {rail[1] * MM} {rail[0] * MM}"/></geometry></visual>'
        f'<inertial><mass value="{1.06 * P.X_RAIL_LEN / 1000:.3f}"/><inertia ixx="1e-3" iyy="1e-3" izz="1e-3" ixy="0" ixz="0" iyz="0"/></inertial></link>'
    )
    for b in bodies:
        c = b.com()
        ixx, iyy, izz = b.inertia()
        vis = "".join(
            f'<visual name="{p.name}"><origin xyz="{_f(x * MM for x in p.pos)}"/><geometry><box size="{_f(s * MM for s in p.size)}"/></geometry></visual>'
            for p in b.parts
        )
        out.append(
            f'  <link name="{b.name}">{vis}<inertial><origin xyz="{_f(x * MM for x in c)}"/><mass value="{b.mass() * G:.6g}"/>'
            f'<inertia ixx="{ixx:.4g}" iyy="{iyy:.4g}" izz="{izz:.4g}" ixy="0" ixz="0" iyz="0"/></inertial></link>'
        )
        parent = b.parent or "world_base"
        rpy = _rpy_from_quat(b.quat)
        origin = f'<origin xyz="{_f(x * MM for x in b.pos)}" rpy="{_f(rpy)}"/>'
        if b.joint is None:
            out.append(f'  <joint name="{b.name}_fixed" type="fixed"><parent link="{parent}"/><child link="{b.name}"/>{origin}</joint>')
            continue
        if b.jtype == "hinge":
            lo, hi = (math.radians(x) for x in b.limit)
            jt, eff, vel = "revolute", 5.0, 6.0
        else:
            lo, hi = (x * MM for x in b.limit)
            jt, eff, vel = "prismatic", 200.0, 0.3
        name = b.joint
        mimic = ""
        if name == "gripper_right":
            mimic = '<mimic joint="gripper_left" multiplier="1" offset="0"/>'
        if name in P.JOINT_SERVOS:
            eff = P.joint_capacity(name)[0]
        out.append(
            f'  <joint name="{name}" type="{jt}"><parent link="{parent}"/><child link="{b.name}"/>{origin}'
            f'<axis xyz="{_f(b.axis)}"/><limit lower="{lo:.4f}" upper="{hi:.4f}" effort="{eff:.3f}" velocity="{vel}"/>{mimic}</joint>'
        )
        for sname, spos in b.sites.items():
            out.append(f'  <link name="{sname}"/><joint name="{sname}_fixed" type="fixed"><parent link="{b.name}"/><child link="{sname}"/><origin xyz="{_f(x * MM for x in spos)}"/></joint>')
    out.append("</robot>")
    return "\n".join(out) + "\n"
