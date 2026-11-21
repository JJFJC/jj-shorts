"""World poses of every MuJoCo body for a joint configuration (used by CAD assembly + renders)."""

import math
import sys
from pathlib import Path

import mujoco

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import model as M  # noqa: E402

HOME = dict(gantry_x=400.0, gantry_z=0.0, base_yaw=0.0, shoulder_lift=0.0, elbow_flex=0.0, wrist_flex=0.0, wrist_roll=0.0)


def body_poses(q: dict) -> dict:
    """q: joint -> degrees (revolute) or mm (prismatic). Returns body -> (pos_mm[3], quat_wxyz[4])."""
    m = mujoco.MjModel.from_xml_string(M.to_mjcf(0.0))
    d = mujoco.MjData(m)
    for name, val in q.items():
        j = m.joint(name)
        d.qpos[m.jnt_qposadr[j.id]] = val / 1000.0 if m.jnt_type[j.id] == mujoco.mjtJoint.mjJNT_SLIDE else math.radians(val)
    mujoco.mj_kinematics(m, d)
    out = {"world": ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])}
    for b in range(1, m.nbody):
        out[m.body(b).name] = ([float(x) * 1000 for x in d.xpos[b]], [float(x) for x in d.xquat[b]])
    return out
