"""Arm v2 (stepper joints on CAN) through the real firmware core, with simulated MKS drivers."""

import pytest

from lerobot_robot_gantry_arm.gantry_driver import GantryDriver, GantryError, parse_jpos


def test_parse_jpos():
    s = parse_jpos("ok JPOS 1.000 -2.500 3.000 0.000 90.000 H=12-45 F=--S--")
    assert s.deg["shoulder_lift"] == -2.5 and s.deg["wrist_roll"] == 90.0
    assert s.homed == [True, True, False, True, True]
    assert s.faults == {"elbow_flex": "S"} and not s.ok


def test_jmove_needs_jhome_then_reaches_the_target(fw):
    d = GantryDriver(fw, timeout_s=5)
    with pytest.raises(GantryError) as e:
        d.jmove({"shoulder_lift": 10})
    assert e.value.code == "NOT_HOMED"
    d.jhome()
    assert "ev JHOMED" in d.events
    goal = {"base_yaw": 25.0, "shoulder_lift": 40.0, "elbow_flex": 60.0, "wrist_flex": -30.0, "wrist_roll": 90.0}
    d.jmove(goal, speed=0.6)
    fw.tick(10.0)
    pos = d.jpos()
    assert pos.ok and all(pos.homed)
    for k, v in goal.items():
        assert pos.deg[k] == pytest.approx(v, abs=0.01)  # within a few encoder counts


def test_joint_limit_rejected_end_to_end(fw):
    d = GantryDriver(fw, timeout_s=5)
    d.jhome()
    with pytest.raises(GantryError) as e:
        d.jmove({"shoulder_lift": 150})  # limit 135
    assert e.value.code == "LIMIT"


def test_collision_stalls_the_joint_and_blocks_motion_until_reset(fw):
    d = GantryDriver(fw, timeout_s=5)
    d.jhome()
    fw.arm_block(2, -60.0)  # something in the elbow's way at -60 deg (it homes at -140)
    d.jmove({"elbow_flex": 0.0}, speed=1.0)
    fw.tick(5.0)
    d.poll_events()
    assert any(ev == "ev JSTALL J3" for ev in d.events)
    pos = d.jpos()
    assert pos.faults == {"elbow_flex": "S"}
    assert pos.deg["elbow_flex"] == pytest.approx(-60.0, abs=0.05)  # stopped at the obstacle
    with pytest.raises(GantryError) as e:
        d.jmove({"elbow_flex": 10.0})
    assert e.value.code == "ARM_FAULT"
    fw.arm_unblock(2)
    d.reset()
    fw.tick(0.2)
    assert d.jpos().ok
    d.jmove({"elbow_flex": 10.0})
    fw.tick(5.0)
    assert d.jpos().deg["elbow_flex"] == pytest.approx(10.0, abs=0.01)


def test_estop_drops_arm_homing(fw):
    d = GantryDriver(fw, timeout_s=5)
    d.jhome()
    d.estop()
    d.reset()
    assert not any(d.jpos().homed)  # unpowered joints can be back-driven: home again
