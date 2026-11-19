import math
import random

import pytest

from lerobot_robot_gantry_arm.kinematics import ASSETS, WORK_AREA, IKError, Kinematics, Pose

mujoco = pytest.importorskip("mujoco")
K = Kinematics()


def mj_tcp(q):
    m = mujoco.MjModel.from_xml_path(str(ASSETS / "gantry_arm.xml"))
    d = mujoco.MjData(m)
    for n, v in q.items():
        j = m.joint(n)
        d.qpos[m.jnt_qposadr[j.id]] = v / 1000 if n.startswith("gantry") else math.radians(v)
    mujoco.mj_kinematics(m, d)
    return d.site("tcp").xpos * 1000


def test_fk_matches_mujoco():
    rng = random.Random(0)
    for _ in range(50):
        q = {"gantry_x": rng.uniform(0, 800), "gantry_z": rng.uniform(0, 500), "base_yaw": rng.uniform(-150, 150),
             "shoulder_lift": rng.uniform(-100, 135), "elbow_flex": rng.uniform(-140, 140), "wrist_flex": rng.uniform(-110, 110),
             "wrist_roll": rng.uniform(-180, 180)}
        assert K.fk(q)[:3] == pytest.approx(tuple(mj_tcp(q)), abs=1e-6)


@pytest.mark.parametrize("x,y,z", [(0, 250, 5), (-500, 200, 30), (400, 300, 60), (100, 380, 150), (0, 150, 200)])
def test_ik_roundtrip_through_mujoco(x, y, z):
    q = K.ik(Pose(x, y, z, tilt=180))
    assert K.within_limits(q) and not K.collides(q)
    assert tuple(mj_tcp(q)) == pytest.approx((x, y, z), abs=1e-6)
    assert K.fk(q)[3] == pytest.approx(180.0, abs=1e-6)  # tool points straight down


def test_table_work_area_is_reachable_tool_down():
    """Pick/place and plotting rely on WORK_AREA (tool straight down, z 5..40 mm)."""
    (x0, x1), (y0, y1) = WORK_AREA["x"], WORK_AREA["y"]
    for x in range(int(x0), int(x1) + 1, 50):
        for y in range(int(y0), int(y1) + 1, 19):
            for z in (5, 40):
                assert K.reachable(Pose(x, y, z, 180)), (x, y, z)


def test_unreachable_and_self_collision_rejected():
    with pytest.raises(IKError):
        K.ik(Pose(0, 900, 5))  # too far
    # straight down right next to J1 would drive the wrist through the lift platform
    q = {"gantry_x": 400, "gantry_z": 0, "base_yaw": 0, "shoulder_lift": 120, "elbow_flex": 120, "wrist_flex": -60, "wrist_roll": 0}
    assert K.collides(q)
