"""Motion layer used by every skill: Cartesian targets -> IK -> smooth joint trajectories.

Works with any GantryArm backend (sim or hardware). Moves are sent at `rate_hz` through
`robot.send_action`, so collision detection, limits and gravity feed-forward stay active.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..kinematics import JOINTS, Kinematics, Pose

ARM_KEYS = JOINTS  # gantry_x, gantry_z, base_yaw .. wrist_roll


@dataclass
class Limits:
    joint_deg_s: float = 60.0
    gantry_x_mm_s: float = 150.0
    gantry_z_mm_s: float = 18.0
    cart_mm_s: float = 60.0  # straight-line tool speed for Cartesian moves (drawing, insertion)


def smoothstep(s: float) -> float:
    """Minimum-jerk time scaling (0->1, zero velocity and acceleration at both ends)."""
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10 - 15 * s + 6 * s * s)


@dataclass
class Motion:
    robot: object
    kin: Kinematics = field(default_factory=Kinematics)
    limits: Limits = field(default_factory=Limits)
    rate_hz: float = 30.0
    trace: list = field(default_factory=list)  # (t, tcp_cmd_xyz, gripper, tcp_actual_xyz|None, q) for plots/tests
    gripper: float = 100.0
    _t: float = 0.0
    q_cmd: dict | None = None  # last commanded joints; planning starts here, not from noisy readings

    # ------------------------------------------------------------------ state
    def measured(self) -> dict:
        obs = self.robot.get_observation()
        return {k: obs[f"{k}.pos"] for k in ARM_KEYS}

    def joints(self) -> dict:
        if self.q_cmd is None:
            self.q_cmd = self.measured()
        return dict(self.q_cmd)

    def _send(self, q: dict):
        self.q_cmd = dict(q)
        act = {f"{k}.pos": v for k, v in q.items()}
        act["gripper.pos"] = self.gripper
        self.robot.send_action(act)
        self.robot.step(1.0 / self.rate_hz)
        self._t += 1.0 / self.rate_hz
        sim = getattr(self.robot, "sim", None)
        actual = sim.tcp_mm() if sim is not None else None
        self.trace.append((self._t, self.kin.fk(q)[:3], self.gripper, actual, dict(q)))

    def _duration(self, a: dict, b: dict) -> float:
        L = self.limits
        t = 0.0
        for k in ARM_KEYS:
            d = abs(b[k] - a[k])
            v = L.gantry_x_mm_s if k == "gantry_x" else L.gantry_z_mm_s if k == "gantry_z" else L.joint_deg_s
            t = max(t, 1.875 * d / v)  # min-jerk peak speed = 1.875 x average
        return max(t, 0.1)

    # ------------------------------------------------------------------ moves
    def move_joints(self, target: dict, settle_s: float = 0.15):
        start = self.joints()
        target = {**start, **target}
        T = self._duration(start, target)
        n = max(1, int(math.ceil(T * self.rate_hz)))
        for i in range(1, n + 1):
            s = smoothstep(i / n)
            self._send({k: start[k] + s * (target[k] - start[k]) for k in ARM_KEYS})
        for _ in range(int(settle_s * self.rate_hz)):
            self._send(target)
        return target

    def move_to(self, pose: Pose, also: list[Pose] | None = None, **ik_kw) -> dict:
        """Point-to-point (joint-interpolated) move to a tool pose.

        `also`: poses the following straight-line moves will visit with the gantry held; the gantry
        position is chosen so all of them are reachable (e.g. the grasp below an approach point).
        """
        if also:
            gx, gz = self.kin.common_gantry([pose, *also], prefer=self.joints())
            ik_kw = {**ik_kw, "gantry_x": gx, "gantry_z": gz}
        q = self.kin.ik(pose, prefer=self.joints(), **ik_kw)
        return self.move_joints(q)

    def line_to(self, pose: Pose, speed_mm_s: float | None = None, step_mm: float = 2.0):
        """Straight-line Cartesian move with the gantry axes held (drawing, insertion, lifting)."""
        q0 = self.joints()
        x0, y0, z0, tilt0 = self.kin.fk(q0)
        dist = math.dist((x0, y0, z0), (pose.x, pose.y, pose.z))
        v = speed_mm_s or self.limits.cart_mm_s
        n = max(1, int(math.ceil(max(dist / step_mm, dist / v * self.rate_hz))))
        q = q0
        for i in range(1, n + 1):
            s = smoothstep(i / n) if n > 3 else i / n
            p = Pose(x0 + s * (pose.x - x0), y0 + s * (pose.y - y0), z0 + s * (pose.z - z0), pose.tilt, pose.roll)
            q = self.kin.ik(p, gantry_x=q0["gantry_x"], gantry_z=q0["gantry_z"])
            self._send(q)
        return q

    def path(self, points: list[tuple[float, float]], z: float, speed_mm_s: float | None = None, step_mm: float = 1.5, roll: float = 0.0):
        """Follow a polyline at constant height (pen plotting). Gantry X may shift between strokes, not within one."""
        if not points:
            return
        q_hold = self.joints()
        v = speed_mm_s or self.limits.cart_mm_s
        for (xa, ya), (xb, yb) in zip(points, points[1:]):
            seg = math.dist((xa, ya), (xb, yb))
            n = max(1, int(math.ceil(max(seg / step_mm, seg / v * self.rate_hz))))
            for i in range(1, n + 1):
                s = i / n
                p = Pose(xa + s * (xb - xa), ya + s * (yb - ya), z, 180.0, roll)
                q_hold = self.kin.ik(p, gantry_x=q_hold["gantry_x"], gantry_z=q_hold["gantry_z"])
                self._send(q_hold)

    def set_gripper(self, opening: float, wait_s: float = 0.6):
        """0 = closed .. 100 = open."""
        self.gripper = opening
        q = self.joints()
        for _ in range(int(wait_s * self.rate_hz)):
            self._send(q)
