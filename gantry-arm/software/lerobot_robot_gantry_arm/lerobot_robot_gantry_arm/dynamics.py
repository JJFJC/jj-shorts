"""Gravity feed-forward, ganged servo pairs and collision detection.

These are the pieces that make cheap position servos behave on a 450 mm arm:

* GravityModel  - static joint torques from the MuJoCo model (CAD masses + shoulder spring)
* feedforward   - offset each goal by tau/kp so the servo's P-loop holds the true target
                  (sim: 37 mm TCP sag -> <1 mm, see docs/SIM_REPORT.md)
* Ganged        - J2 and J3 are each two STS3250 on one axis: one action key, two IDs,
                  mismatch watchdog so the pair never fights itself
* CollisionMonitor - measured servo load vs. the load gravity alone explains
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

ASSETS = Path(__file__).resolve().parent / "assets"
ARM_JOINTS = ["base_yaw", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# SO-101 measured STS3215 position stiffness at P_Coefficient=16 (TheRobotStudio, Apache-2.0)
KP_STS3215_P16_NM_PER_RAD = 17.8


def servo_kp_nm_per_deg(stall_nm: float, count: int, p_coefficient: int, ratio: float = 1.0) -> float:
    """Joint-side position-loop stiffness of a (possibly ganged, possibly geared) joint, scaled from the
    SO-101 measurement. A gear ratio n multiplies the stiffness seen at the joint by n^2."""
    k_rad = KP_STS3215_P16_NM_PER_RAD * (p_coefficient / 16.0) * (stall_nm / 2.942) * count * ratio * ratio
    return k_rad * math.pi / 180.0


class GravityModel:
    """Static torque (N·m) each arm joint must supply to hold a pose, incl. the shoulder spring."""

    def __init__(self, mjcf: Path | None = None, payload_g: float = 0.0):
        import mujoco  # optional dependency ([sim] extra)

        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(mjcf or ASSETS / "gantry_arm.xml"))
        self.d = mujoco.MjData(self.m)
        self._payload_id = self.m.body("payload").id
        self.set_payload(payload_g)

    def set_payload(self, grams: float):
        self.m.body_mass[self._payload_id] = max(0.0, grams) / 1000.0

    def torques(self, q_deg: dict) -> dict:
        m, d = self.m, self.d
        d.qpos[:] = 0
        d.qvel[:] = 0
        d.qacc[:] = 0
        for j in ARM_JOINTS:
            if j in q_deg:
                d.qpos[m.jnt_qposadr[m.joint(j).id]] = math.radians(q_deg[j])
        self.mj.mj_inverse(m, d)
        return {j: float(d.qfrc_inverse[m.jnt_dofadr[m.joint(j).id]]) for j in ARM_JOINTS}


def feedforward(goal_deg: dict, tau_nm: dict, kp_nm_per_deg: dict, gain: float = 1.0, max_offset_deg: float = 8.0) -> dict:
    """Goal + tau/kp for each joint, clamped. A P-servo settles where kp*(goal-q) = tau."""
    out = dict(goal_deg)
    for j, tau in tau_nm.items():
        if j in out and j in kp_nm_per_deg and kp_nm_per_deg[j] > 0:
            off = gain * tau / kp_nm_per_deg[j]
            out[j] = goal_deg[j] + max(-max_offset_deg, min(max_offset_deg, off))
    return out


@dataclass
class Ganged:
    """Two servos on one joint. The secondary is mounted mirrored; calibration (drive_mode=1)
    makes both read the same joint angle, so both get the same goal."""

    pairs: dict = field(default_factory=lambda: {"shoulder_lift": "shoulder_lift_2", "elbow_flex": "elbow_flex_2"})
    tolerance_deg: float = 3.0
    cycles: int = 5
    _bad: dict = field(default_factory=dict)

    def expand(self, goals: dict) -> dict:
        out = dict(goals)
        for a, b in self.pairs.items():
            if a in goals:
                out[b] = goals[a]
        return out

    def merge(self, readings: dict) -> dict:
        """Report the pair as one joint (mean of the two encoders)."""
        out = {k: v for k, v in readings.items() if k not in self.pairs.values()}
        for a, b in self.pairs.items():
            if a in readings and b in readings:
                out[a] = 0.5 * (readings[a] + readings[b])
        return out

    def check(self, readings: dict) -> list[str]:
        """Joints whose two servos disagree by > tolerance for `cycles` consecutive reads."""
        faults = []
        for a, b in self.pairs.items():
            if a not in readings or b not in readings:
                continue
            if abs(readings[a] - readings[b]) > self.tolerance_deg:
                self._bad[a] = self._bad.get(a, 0) + 1
            else:
                self._bad[a] = 0
            if self._bad[a] >= self.cycles:
                faults.append(a)
        return faults


@dataclass
class GearedPair:
    """Two servos driving one joint through their own pinions on a shared hub gear (docs/MECHANICS.md).

    Motor angle = ratio x joint angle, with a DIFFERENT ratio per servo (vernier: e.g. 36/18 and 36/17).
    - expand():   joint goals -> motor goals, the two servos offset +/-preload so they push against each
                  other and the gear backlash is taken out.
    - merge():    motor readings -> one joint angle (mean of the two, each divided by its own ratio).
    - check():    the two servos disagree on the joint angle (loose pinion, skipped tooth) -> fault.
    - absolute(): at power-up each servo only knows its angle within one turn; the two different ratios
                  pick the one joint angle that fits both readings (like the vernier on a caliper).
    - unwrap():   track motor turns across the +/-180 deg wrap of single-turn readings.
    Same interface as Ganged, so the robot class can use either.
    """

    ratios: dict = field(default_factory=lambda: {"shoulder_lift": (2.0, 36 / 17), "elbow_flex": (2.0, 30 / 14)})
    preload_deg: dict = field(default_factory=lambda: {"shoulder_lift": 0.4, "elbow_flex": 0.4})  # motor-side, per servo
    pairs: dict = field(default_factory=lambda: {"shoulder_lift": "shoulder_lift_2", "elbow_flex": "elbow_flex_2"})
    tolerance_deg: float = 3.0  # joint-side disagreement that counts as a fault
    cycles: int = 5
    margin_deg: float = 6.0  # vernier: best candidate must beat the next by this much (motor deg)
    max_error_deg: float = 4.0  # ... and fit servo B to within this (backlash + encoder + calibration)
    _bad: dict = field(default_factory=dict)
    _turns: dict = field(default_factory=dict)  # motor -> whole turns added by unwrap()
    _last: dict = field(default_factory=dict)

    def expand(self, goals: dict) -> dict:
        out = dict(goals)
        for a, b in self.pairs.items():
            if a in goals:
                ra, rb = self.ratios[a]
                d = self.preload_deg.get(a, 0.0)
                out[a] = ra * goals[a] + d
                out[b] = rb * goals[a] - d
        return out

    def _joint_pair(self, a, readings):
        b = self.pairs[a]
        ra, rb = self.ratios[a]
        return readings[a] / ra, readings[b] / rb

    def merge(self, readings: dict) -> dict:
        out = {k: v for k, v in readings.items() if k not in self.pairs.values()}
        for a, b in self.pairs.items():
            if a in readings and b in readings:
                qa, qb = self._joint_pair(a, readings)
                out[a] = 0.5 * (qa + qb)
        return out

    def check(self, readings: dict) -> list[str]:
        faults = []
        for a, b in self.pairs.items():
            if a not in readings or b not in readings:
                continue
            qa, qb = self._joint_pair(a, readings)
            self._bad[a] = self._bad.get(a, 0) + 1 if abs(qa - qb) > self.tolerance_deg else 0
            if self._bad[a] >= self.cycles:
                faults.append(a)
        return faults

    def absolute(self, joint: str, a_deg: float, b_deg: float, lo: float, hi: float) -> float:
        """Joint angle from the two single-turn readings (motor deg in [-180, 180), zero = calibration pose)."""
        ra, rb = self.ratios[joint]
        cands = []
        k = math.floor((ra * lo - a_deg) / 360.0) - 1
        while True:
            q = (a_deg + 360.0 * k) / ra
            k += 1
            if q < lo - 1.0:
                continue
            if q > hi + 1.0:
                break
            e = (rb * q - b_deg + 180.0) % 360.0 - 180.0
            cands.append((abs(e), q))
        if not cands:
            raise ValueError(f"{joint}: no joint angle in [{lo}, {hi}] fits the servo readings")
        cands.sort()
        if cands[0][0] > self.max_error_deg:
            raise ValueError(f"{joint}: servo readings disagree by {cands[0][0]:.1f} deg (loose pinion or bad zero calibration)")
        if len(cands) > 1 and cands[1][0] - cands[0][0] < self.margin_deg:
            raise ValueError(f"{joint}: vernier ambiguous ({cands[0][0]:.1f} vs {cands[1][0]:.1f} deg); check the zero calibration")
        return cands[0][1]

    def seed(self, joint: str, q: float, a_deg: float, b_deg: float):
        """After absolute(): set the turn counters so unwrap() returns ratio x q for each servo."""
        b = self.pairs[joint]
        ra, rb = self.ratios[joint]
        for m, r, raw in ((joint, ra, a_deg), (b, rb, b_deg)):
            self._turns[m] = round((r * q - raw) / 360.0)
            self._last[m] = raw

    def unwrap(self, readings: dict) -> dict:
        """Single-turn readings (deg, wrapping at +/-180) -> multi-turn motor angles for the paired servos."""
        out = dict(readings)
        for m in [*self.pairs, *self.pairs.values()]:
            if m not in readings:
                continue
            raw = readings[m]
            if m in self._last:
                d = raw - self._last[m]
                if d > 180.0:
                    self._turns[m] = self._turns.get(m, 0) - 1
                elif d < -180.0:
                    self._turns[m] = self._turns.get(m, 0) + 1
            self._last[m] = raw
            out[m] = raw + 360.0 * self._turns.get(m, 0)
        return out


def decode_load(raw: int) -> float:
    """STS Present_Load register -> signed percent of max torque (bit 10 = direction, 0.1 % units)."""
    raw = int(raw)
    if raw < 0:  # already sign-decoded by the bus
        return raw / 10.0
    mag = (raw & 0x3FF) / 10.0
    return -mag if raw & 0x400 else mag


@dataclass
class CollisionMonitor:
    """Flags a joint that is *stalled against something*.

    A position servo's load spikes whenever it accelerates toward a new goal, so load alone
    gives false alarms. A collision is: load well above what gravity explains (GravityModel),
    AND a large tracking error, AND the joint is not moving - for `cycles` consecutive checks.
    Loads are in percent of the joint's total stall torque.
    """

    threshold_pct: float = 22.0
    cycles: int = 3
    min_error_deg: float = 3.0
    max_speed_deg_s: float = 4.0
    sign: dict = field(default_factory=dict)  # per-joint load sign (servo vs model), default +1
    _count: dict = field(default_factory=dict)

    def update(self, expected_pct: dict, measured_pct: dict, error_deg: dict, speed_deg_s: dict) -> str | None:
        hit = None
        for j, meas in measured_pct.items():
            if j not in expected_pct:
                continue
            resid = abs(meas - self.sign.get(j, 1.0) * expected_pct[j])
            stalled = abs(error_deg.get(j, 0.0)) > self.min_error_deg and abs(speed_deg_s.get(j, 0.0)) < self.max_speed_deg_s
            self._count[j] = self._count.get(j, 0) + 1 if (resid > self.threshold_pct and stalled) else 0
            if self._count[j] >= self.cycles and hit is None:
                hit = j
        return hit

    def reset(self):
        self._count.clear()
