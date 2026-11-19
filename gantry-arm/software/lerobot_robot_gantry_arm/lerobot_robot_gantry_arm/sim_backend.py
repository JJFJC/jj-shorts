"""MuJoCo stand-in for the hardware, so the robot class, skills and policies run with no robot.

Same physics model as the design analysis (assets/gantry_arm.xml: CAD masses, servo stiffness,
shoulder spring). Servos are MuJoCo position actuators with the measured STS stiffness, so the
sim sags like the real arm and gravity feed-forward is exercised for real.
"""

from __future__ import annotations

import math
from pathlib import Path

ASSETS = Path(__file__).resolve().parent / "assets"
ARM = ["base_yaw", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
FINGER_STROKE_M = 0.020


class SimBackend:
    def __init__(self, mjcf: Path | None = None, payload_g: float = 0.0, feed_mm_s: float = 150.0, p_scale: dict | None = None):
        import mujoco

        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(mjcf or ASSETS / "gantry_arm.xml"))
        self.d = mujoco.MjData(self.m)
        self.m.body_mass[self.m.body("payload").id] = payload_g / 1000.0
        self.act = {self.m.actuator(i).name: i for i in range(self.m.nu)}
        self.qadr = {self.m.joint(i).name: self.m.jnt_qposadr[i] for i in range(self.m.njnt)}
        self.feed = {"gantry_x": feed_mm_s, "gantry_z": 20.0}
        self.g_target = {"gantry_x": 400.0, "gantry_z": 250.0}
        self.g_cmd = dict(self.g_target)
        # servo P_Coefficient relative to 16 (the gain the model's kp was measured at)
        for j, k in (p_scale or {}).items():
            i = self.act[j]
            self.m.actuator_gainprm[i, 0] *= k
            self.m.actuator_biasprm[i, 1] *= k
        self.tool_id = 1  # gripper mounted
        self.estop = False

    # ------------------------------------------------------------------ lifecycle
    def connect(self):
        d = self.d
        d.qpos[:] = 0
        for g, v in self.g_cmd.items():
            d.qpos[self.qadr[g]] = v / 1000.0
            d.ctrl[self.act[g]] = v / 1000.0
        d.ctrl[self.act["gripper"]] = FINGER_STROKE_M
        self.mj.mj_forward(self.m, d)
        self.step(0.5)

    # ------------------------------------------------------------------ arm
    def read_arm(self) -> dict:
        out = {j: math.degrees(self.d.qpos[self.qadr[j]]) for j in ARM}
        out["gripper"] = 100.0 * self.d.qpos[self.qadr["gripper_left"]] / FINGER_STROKE_M
        return out

    def write_arm(self, goals: dict):
        if self.estop:
            return
        for j, v in goals.items():
            if j == "gripper":
                self.d.ctrl[self.act["gripper"]] = max(0.0, min(100.0, v)) / 100.0 * FINGER_STROKE_M
            elif j in self.act:
                self.d.ctrl[self.act[j]] = math.radians(v)

    def read_loads_pct(self) -> dict:
        out = {}
        for j in ARM:
            i = self.act[j]
            out[j] = 100.0 * self.d.actuator_force[i] / self.m.actuator_forcerange[i][1]
        return out

    # ------------------------------------------------------------------ gantry (mm)
    def gantry_move(self, x: float | None = None, z: float | None = None):
        if self.estop:
            return
        if x is not None:
            self.g_target["gantry_x"] = min(max(x, 0.0), 800.0)
        if z is not None:
            self.g_target["gantry_z"] = min(max(z, 0.0), 500.0)

    def gantry_pos(self) -> dict:
        return {g: self.d.qpos[self.qadr[g]] * 1000.0 for g in ("gantry_x", "gantry_z")}

    # ------------------------------------------------------------------ time
    def step(self, seconds: float):
        dt = self.m.opt.timestep
        for _ in range(max(1, int(round(seconds / dt)))):
            for g in ("gantry_x", "gantry_z"):  # trapezoid-free constant-feed profile
                err = self.g_target[g] - self.g_cmd[g]
                stepmm = self.feed[g] * dt
                self.g_cmd[g] += max(-stepmm, min(stepmm, err))
                self.d.ctrl[self.act[g]] = self.g_cmd[g] / 1000.0
            self.mj.mj_step(self.m, self.d)

    def tcp_mm(self) -> tuple[float, float, float]:
        p = self.d.site("tcp").xpos
        return (p[0] * 1000.0, p[1] * 1000.0, p[2] * 1000.0)
