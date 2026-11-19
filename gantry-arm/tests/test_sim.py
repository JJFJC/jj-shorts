"""Cross-checks the MuJoCo torque numbers against independent hand calculations."""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "sim")]

mujoco = pytest.importorskip("mujoco")
import model as M  # noqa: E402
import params as P  # noqa: E402
import static_check as S  # noqa: E402

g = 9.81


def hand_shoulder_torque(payload: bool) -> float:
    """Sum of m*g*lever for everything outboard of J2, arm horizontal, from params only."""
    A, Mm = P.ARM, P.MASS
    c, t = A["coupler"], A["tool_tcp"]
    j3, j4, j5 = A["upper"], A["upper"] + A["fore"], A["upper"] + A["fore"] + A["wrist"]
    items = [
        (Mm["upper"], A["upper"] / 2), (P.STS3250.mass_g, j3),
        (Mm["fore"], j3 + A["fore"] / 2), (P.STS3215.mass_g, j4),
        (Mm["wrist"], j4 + A["wrist"] * 0.5), (P.STS3215.mass_g, j4 + A["wrist"] * 0.45),
        (Mm["coupler_robot"], j5 + c * 0.25), (Mm["coupler_tool"], j5 + c * 0.75),
        (P.STS3215.mass_g, j5 + c + 22), (Mm["gripper_parts"], j5 + c + t * 0.55),
        (8.0, j5 + c + t - 10 + 5), (8.0, j5 + c + t - 10 + 5),  # fingers
    ]
    if payload:
        items.append((Mm["payload"], j5 + c + t))
    return sum(m / 1000 * g * L / 1000 for m, L in items)


@pytest.mark.parametrize("payload", [True, False])
def test_shoulder_torque_matches_hand_calc(payload):
    arm = S.Arm(0.0)
    arm.set_payload(payload)
    tq = arm.torques(S.FULL_REACH)
    assert abs(tq["shoulder_lift"]) == pytest.approx(hand_shoulder_torque(payload), rel=0.01)


def test_spring_torque_is_k_a_b_sin_theta():
    k = 0.8
    with_s = S.Arm(k).torques(S.FULL_REACH)["shoulder_lift"]
    without = S.Arm(0.0).torques(S.FULL_REACH)["shoulder_lift"]
    expected = k * 1000 * P.SPRING["a"] / 1000 * P.SPRING["b"] / 1000 * math.sin(math.radians(90))
    assert with_s - without == pytest.approx(expected, rel=1e-3)


def test_vertical_arm_needs_no_pitch_torque():
    tq = S.Arm(0.0).torques({"shoulder_lift": 0, "elbow_flex": 0, "wrist_flex": 0})
    for j in ("shoulder_lift", "elbow_flex", "wrist_flex"):
        assert abs(tq[j]) < 1e-6


def test_reach_matches_params():
    arm = S.Arm(0.0)
    arm.set_pose(S.FULL_REACH)
    turret_y = None
    tcp = arm.tcp()
    turret_y = arm.d.xpos[arm.m.body("turret").id][1]
    assert (tcp[1] - turret_y) * 1000 == pytest.approx(P.REACH, abs=0.5)


def test_urdf_and_mjcf_agree_on_mass_and_joints():
    urdf = ET.fromstring(M.to_urdf())
    m = mujoco.MjModel.from_xml_string(M.to_mjcf(0.0))
    urdf_mass = sum(float(l.find("inertial/mass").get("value")) for l in urdf.findall("link") if l.find("inertial") is not None and l.get("name") != "world_base")
    assert urdf_mass == pytest.approx(sum(m.body_mass), rel=1e-6)
    moving = {j.get("name"): j.get("type") for j in urdf.findall("joint") if j.get("type") != "fixed"}
    assert sum(t == "prismatic" for t in moving.values()) == 4  # X, Z, 2 fingers
    assert sum(t == "revolute" for t in moving.values()) == 5
    for j in urdf.findall("joint"):
        if j.get("type") != "fixed":
            assert j.find("limit") is not None


def test_t8x2_is_self_locking_and_t8x8_is_not():
    def locks(lead, mu=P.Z_SCREW["mu_nut"], d=8.0):
        dm = d - lead / 2 if lead <= 2 else 7.0
        return math.atan(lead / (math.pi * dm)) < math.atan(mu / math.cos(math.radians(15)))

    assert locks(2.0)
    assert not locks(8.0)
