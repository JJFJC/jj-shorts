"""Forward / inverse kinematics for the Gantry Arm (2 prismatic + 5 revolute).

Conventions (match sim/model.py and the MJCF in assets/):
    world: +X along the rail, +Y out over the work area, +Z up; millimetres, degrees.
    joint zero = arm pointing straight up. shoulder/elbow/wrist pitch are measured from
    vertical, positive tilting the arm forward (away from the column).
    base_yaw = 0 points the arm along +Y.

IK is analytic: the redundant gantry axes are chosen by a small search that keeps the
elbow well-conditioned, then a planar 2-link solve gives shoulder/elbow and the wrist
takes up the requested tool angle.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

ASSETS = Path(__file__).resolve().parent / "assets"
# Table rectangle reachable everywhere with the tool pointing straight down, z = 5..40 mm
# (measured; enforced by tests/test_kinematics.py). World mm.
WORK_AREA = {"x": (-450.0, 450.0), "y": (110.0, 300.0)}
JOINTS = ["gantry_x", "gantry_z", "base_yaw", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


@dataclass(frozen=True)
class Geometry:
    x_travel: float
    z_travel: float
    x0: float
    j1_y: float
    j2_z0: float
    upper: float
    fore: float
    tool: float
    lateral: float
    limits_deg: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path | None = None) -> "Geometry":
        d = json.loads((path or ASSETS / "geometry.json").read_text())
        keep = {k: d[k] for k in cls.__dataclass_fields__}
        keep["limits_deg"] = {k: tuple(v) for k, v in d["limits_deg"].items()}
        return cls(**keep)


@dataclass
class Pose:
    """Tool pose target. `tilt` = tool axis angle from vertical-up in the arm plane (180 = pointing down)."""

    x: float
    y: float
    z: float
    tilt: float = 180.0
    roll: float = 0.0


class IKError(ValueError):
    pass


class Kinematics:
    def __init__(self, geo: Geometry | None = None):
        self.g = geo or Geometry.load()

    # ------------------------------------------------------------------ FK
    def fk(self, q: dict) -> tuple[float, float, float, float]:
        """Joint dict (mm / deg) -> TCP (x, y, z) in mm and tool tilt from vertical (deg)."""
        g = self.g
        psi = math.radians(q.get("base_yaw", 0.0))
        a1 = math.radians(q.get("shoulder_lift", 0.0))
        a2 = a1 + math.radians(q.get("elbow_flex", 0.0))
        a3 = a2 + math.radians(q.get("wrist_flex", 0.0))
        r = g.upper * math.sin(a1) + g.fore * math.sin(a2) + g.tool * math.sin(a3)
        z = g.j2_z0 + q.get("gantry_z", 0.0) + g.upper * math.cos(a1) + g.fore * math.cos(a2) + g.tool * math.cos(a3)
        jx, jy = g.x0 + q.get("gantry_x", 0.0), g.j1_y
        # forward f = (-sin psi, cos psi); lateral offset along s = (-cos psi, -sin psi) * (-lateral)
        fx, fy = -math.sin(psi), math.cos(psi)
        sx, sy = math.cos(psi), math.sin(psi)
        x = jx + r * fx + g.lateral * sx
        y = jy + r * fy + g.lateral * sy
        return x, y, z, math.degrees(a3)

    # ------------------------------------------------------------------ IK
    def within_limits(self, q: dict) -> bool:
        g = self.g
        if not (-1e-6 <= q["gantry_x"] <= g.x_travel + 1e-6 and -1e-6 <= q["gantry_z"] <= g.z_travel + 1e-6):
            return False
        for j, (lo, hi) in g.limits_deg.items():
            if j in q and not (lo - 1e-6 <= q[j] <= hi + 1e-6):
                return False
        return True

    # ------------------------------------------------------------------ self-collision (coarse)
    PLATFORM_R = 75.0    # platform round end + J1 base housing around the J1 axis (mm)
    PLATFORM_DROP = 85.0  # platform plate + ribs below the platform top
    BASE_H = 40.0         # J1 base housing + collar above the platform top
    LINK_R = {"upper": 46.0, "fore": 40.0, "tool": 30.0}

    def chain_points(self, q: dict) -> list[tuple[float, float, float]]:
        """(r_forward, z_world, radius) samples along the arm in its own plane."""
        g = self.g
        a1 = math.radians(q["shoulder_lift"])
        a2 = a1 + math.radians(q["elbow_flex"])
        a3 = a2 + math.radians(q["wrist_flex"])
        z0 = g.j2_z0 + q["gantry_z"]
        pts, r, z = [], 0.0, z0
        for L, a, rad in ((g.upper, a1, self.LINK_R["upper"]), (g.fore, a2, self.LINK_R["fore"]), (g.tool, a3, self.LINK_R["tool"])):
            n = max(2, int(L // 15))
            for i in range(1, n + 1):
                pts.append((r + L * i / n * math.sin(a), z + L * i / n * math.cos(a), rad * (0.75 if i < n else 1.0)))
            r, z = r + L * math.sin(a), z + L * math.cos(a)
        return pts

    def collides(self, q: dict) -> bool:
        """True if the elbow, wrist or tool would hit the lift platform / J1 base or swing behind into the column."""
        g = self.g
        plat = g.j2_z0 - 120.0 + q["gantry_z"]  # platform top (J2 sits 120 mm above it)
        for r, z, rad in self.chain_points(q):
            if z - rad < plat + self.BASE_H and z + rad > plat - self.PLATFORM_DROP and r - rad < self.PLATFORM_R:
                return True
            if r < -40.0 and z - rad < plat + self.BASE_H:  # behind J1: column, Z shroud
                return True
        return False

    def _arm(self, p: Pose, gx: float, gz: float) -> dict | None:
        g = self.g
        jx, jy = g.x0 + gx, g.j1_y
        dx, dy = p.x - jx, p.y - jy
        d2 = dx * dx + dy * dy
        if d2 < g.lateral**2 + 1e-9:
            return None
        rho = math.sqrt(d2 - g.lateral**2)
        # d = R(psi) @ (lateral, rho) in the (x, y) plane  (see fk)
        psi = math.atan2(dy, dx) - math.atan2(rho, g.lateral)
        psi = (psi + math.pi) % (2 * math.pi) - math.pi
        a3 = math.radians(p.tilt)
        wr = rho - g.tool * math.sin(a3)
        wz = p.z - (g.j2_z0 + gz) - g.tool * math.cos(a3)
        D2 = wr * wr + wz * wz
        L1, L2 = g.upper, g.fore
        c3 = (D2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        if c3 > 1.0 or c3 < -1.0:
            return None
        t3 = math.acos(c3)  # elbow bends forward ("elbow up" for reach-down poses)
        t2 = math.atan2(wr, wz) - math.atan2(L2 * math.sin(t3), L1 + L2 * math.cos(t3))
        t4 = a3 - t2 - t3
        t4 = (t4 + math.pi) % (2 * math.pi) - math.pi
        return {
            "gantry_x": gx, "gantry_z": gz,
            "base_yaw": math.degrees(psi), "shoulder_lift": math.degrees(t2), "elbow_flex": math.degrees(t3),
            "wrist_flex": math.degrees(t4), "wrist_roll": p.roll,
        }

    def ik(self, p: Pose, gantry_x: float | None = None, gantry_z: float | None = None, prefer: dict | None = None) -> dict:
        """Solve for a TCP pose. Pass gantry_x / gantry_z to pin those axes; otherwise they are chosen.

        `prefer` (a current joint dict) biases the choice toward small gantry moves.
        """
        g = self.g
        if gantry_x is not None:
            xs = [gantry_x]
        else:
            ideal = p.x - g.x0 - g.lateral  # puts the target straight ahead of J1 (yaw ~ 0)
            xs = sorted({min(max(ideal, 0.0), g.x_travel)} | {min(max(ideal + d, 0.0), g.x_travel) for d in (-150, -75, 75, 150)})
        zs = [gantry_z] if gantry_z is not None else [float(z) for z in range(0, int(g.z_travel) + 1, 10)]
        best, best_cost = None, math.inf
        for gx in xs:
            for gz in zs:
                q = self._arm(p, gx, gz)
                if q is None or not self.within_limits(q) or self.collides(q):
                    continue
                # cost: keep the elbow near 90 deg (well-conditioned), yaw small, wrist mid-range
                cost = (abs(q["elbow_flex"] - 90.0) / 90.0) ** 2 + (q["base_yaw"] / 150.0) ** 2 + (q["wrist_flex"] / 110.0) ** 2
                if prefer:
                    cost += 0.5 * (abs(gx - prefer.get("gantry_x", gx)) / g.x_travel + abs(gz - prefer.get("gantry_z", gz)) / g.z_travel)
                if cost < best_cost:
                    best, best_cost = q, cost
        if best is None:
            raise IKError(f"unreachable: {p}")
        return best

    def common_gantry(self, poses: list[Pose], prefer: dict | None = None, z_min: float = 0.0) -> tuple[float, float]:
        """One (gantry_x, gantry_z) from which every pose is reachable (strokes, insertions).

        Raises IKError if no single gantry position covers them all (caller splits the path).
        """
        g = self.g
        cx = sum(p.x for p in poses) / len(poses)
        ideal = cx - g.x0 - g.lateral
        xs = sorted({min(max(ideal + d, 0.0), g.x_travel) for d in (0, -40, 40, -90, 90, -150, 150)})
        best, best_cost = None, math.inf
        for gx in xs:
            for gz in range(int(math.ceil(z_min / 10.0)) * 10, int(g.z_travel) + 1, 10):
                cost = 0.0
                for p in poses:
                    q = self._arm(p, gx, float(gz))
                    if q is None or not self.within_limits(q) or self.collides(q):
                        cost = math.inf
                        break
                    cost += (abs(q["elbow_flex"] - 90.0) / 90.0) ** 2 + (q["base_yaw"] / 150.0) ** 2
                if prefer and cost < math.inf:
                    cost += len(poses) * 0.5 * abs(gx - prefer.get("gantry_x", gx)) / g.x_travel
                if cost < best_cost:
                    best, best_cost = (gx, float(gz)), cost
        if best is None:
            raise IKError(f"no common gantry position for {len(poses)} poses")
        return best

    def reachable(self, p: Pose, **kw) -> bool:
        try:
            self.ik(p, **kw)
            return True
        except IKError:
            return False
