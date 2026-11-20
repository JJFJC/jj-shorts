"""GantryArm end-to-end. Hardware boundaries only are replaced:
- the Feetech servo bus -> FakeBus (records writes, serves reads)
- the ESP32 serial port -> the real firmware core via tests/firmware_bridge.py
The sim backend runs the real MuJoCo model.
"""

import logging
from contextlib import contextmanager

import pytest

from lerobot_robot_gantry_arm import GantryArm, GantryArmConfig
from lerobot_robot_gantry_arm.gantry_arm import CollisionError

pytest.importorskip("mujoco")
logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)

FULL_REACH = {"shoulder_lift.pos": 90.0, "elbow_flex.pos": 0.0, "wrist_flex.pos": 0.0}


def drive(robot, action, steps=60, dt=0.05):
    for _ in range(steps):
        robot.send_action(action)
        robot.step(dt)
    return robot.get_observation()


@pytest.fixture
def sim_robot(tmp_path):
    made = []

    def make(**kw):
        r = GantryArm(GantryArmConfig(backend="sim", id="t", calibration_dir=tmp_path, **kw))
        r.connect()
        made.append(r)
        return r

    yield make
    for r in made:
        r.disconnect()


# ------------------------------------------------------------------ sim backend


def test_features_cover_gantry_arm_and_gripper(sim_robot):
    r = sim_robot()
    keys = set(r.action_features)
    assert {"gantry_x.pos", "gantry_z.pos", "shoulder_lift.pos", "gripper.pos"} <= keys
    assert "shoulder_lift_2.pos" not in keys  # the twin servo is hidden behind one joint
    assert set(r.get_observation()) == set(r.observation_features)


def test_gravity_feedforward_removes_sag_at_full_reach(sim_robot):
    on = drive(sim_robot(payload_g=500), FULL_REACH)
    off = drive(sim_robot(payload_g=500, gravity_feedforward=False), FULL_REACH)
    for j in ("shoulder_lift", "elbow_flex", "wrist_flex"):
        assert abs(on[f"{j}.pos"] - FULL_REACH[f"{j}.pos"]) < 0.2
    assert abs(off["elbow_flex.pos"] - FULL_REACH["elbow_flex.pos"]) > 0.2  # without it the (geared, 4x stiffer) joint still sags measurably


def test_action_is_clipped_to_limits_and_step_size(sim_robot):
    r = sim_robot()
    applied = r.send_action({"shoulder_lift.pos": 400.0, "gantry_x.pos": -50.0})
    assert applied["shoulder_lift.pos"] <= 15.0 + 0.05  # max_relative_target from ~0 deg (sim noise)
    assert applied["gantry_x.pos"] >= 0.0


def test_gantry_moves_in_sim(sim_robot):
    r = sim_robot()
    obs = drive(r, {"gantry_x.pos": 300.0, "gantry_z.pos": 200.0}, steps=80)
    assert obs["gantry_x.pos"] == pytest.approx(300.0, abs=1.0)
    assert obs["gantry_z.pos"] == pytest.approx(200.0, abs=1.0)


def test_collision_stops_the_arm(sim_robot):
    r = sim_robot(max_relative_target=None)
    drive(r, {"shoulder_lift.pos": 30.0, "elbow_flex.pos": 60.0, "wrist_flex.pos": 60.0}, steps=30)
    # something blocks the forearm: a large external force pushing back against the motion
    fa = r.sim.m.body("forearm").id
    r.sim.d.xfrc_applied[fa, :3] = [0.0, -400.0, 200.0]
    with pytest.raises(CollisionError):
        drive(r, {"shoulder_lift.pos": 80.0, "elbow_flex.pos": 60.0, "wrist_flex.pos": 60.0}, steps=40)
    assert r.collided_joint is not None


def test_normal_fast_motion_is_not_a_collision(sim_robot):
    r = sim_robot(payload_g=500)
    for target in (90.0, -60.0, 120.0, 0.0):
        drive(r, {"shoulder_lift.pos": target, "elbow_flex.pos": 40.0, "wrist_flex.pos": 30.0}, steps=50)


# ------------------------------------------------------------------ hardware path (fake bus + real firmware)


class FakeBus:
    def __init__(self, port, motors, calibration=None):
        self.motors = motors
        self.pos = {m: 0.0 for m in motors}
        self.load = {m: 0 for m in motors}
        self.writes: list[tuple] = []
        self.is_calibrated = True

    def connect(self):
        pass

    def disconnect(self, disable_torque=True):
        pass

    @contextmanager
    def torque_disabled(self):
        yield

    def configure_motors(self):
        pass

    def set_operating_mode(self, mode, motor):
        pass

    def write(self, reg, motor, value):
        self.writes.append((reg, motor, value))

    def sync_read(self, reg, normalize=True, **kw):
        return dict(self.pos) if reg == "Present_Position" else dict(self.load)

    def sync_write(self, reg, values, **kw):
        self.writes.append((reg, dict(values)))
        self.pos.update(values)  # an ideal servo


@pytest.fixture
def hw_robot(tmp_path, monkeypatch, fw):
    import lerobot.motors.feetech as feetech

    from lerobot_robot_gantry_arm import gantry_driver

    monkeypatch.setattr(feetech, "FeetechMotorsBus", FakeBus)
    monkeypatch.setattr(gantry_driver.GantryDriver, "open", classmethod(lambda cls, port, **kw: cls(fw, timeout_s=5)))
    r = GantryArm(GantryArmConfig(backend="hardware", id="hw", calibration_dir=tmp_path, collision_detection=False))
    r.connect()
    yield r
    r.disconnect()


def test_configure_writes_heavy_gains_to_both_twin_servos(hw_robot):
    w = {(reg, m): v for reg, m, v in [x for x in hw_robot.bus.writes if len(x) == 3]}
    for m in ("shoulder_lift", "shoulder_lift_2", "elbow_flex", "elbow_flex_2", "wrist_flex"):
        assert w[("P_Coefficient", m)] == 32
    assert w[("P_Coefficient", "base_yaw")] == 16
    assert w[("Max_Torque_Limit", "gripper")] == 500


def test_send_action_drives_geared_pairs_with_preload_and_feedforward(hw_robot):
    hw_robot.send_action({"shoulder_lift.pos": 10.0, "elbow_flex.pos": 10.0, "gripper.pos": 50.0})
    reg, goals = [w for w in hw_robot.bus.writes if w[0] == "Goal_Position"][-1]
    g = hw_robot.ganged  # geared pairs: each servo gets ratio x joint goal, +/- the anti-backlash preload
    joint = {}
    for j in ("shoulder_lift", "elbow_flex"):
        ra, rb = g.ratios[j]
        d = g.preload_deg[j]
        qa, qb = (goals[j] - d) / ra, (goals[f"{j}_2"] + d) / rb
        assert qa == pytest.approx(qb, abs=1e-9)  # both servos aim at the same joint angle
        assert goals[j] - ra * qa == pytest.approx(d) and goals[f"{j}_2"] - rb * qb == pytest.approx(-d)  # pushing apart
        joint[j] = qa
    # the joint goal is offset in the direction of the static torque the model predicts
    # (at small angles the shoulder spring over-balances, so J2 is pulled back)
    tau = hw_robot.gravity.torques({"shoulder_lift": 10.0, "elbow_flex": 10.0, "base_yaw": 0, "wrist_flex": 0, "wrist_roll": 0})
    for j in ("shoulder_lift", "elbow_flex"):
        assert (joint[j] - 10.0) * tau[j] > 0
    assert goals["gripper"] == 50.0


def test_gantry_goes_through_the_firmware(hw_robot, fw):
    hw_robot.send_action({"gantry_x.pos": 120.0, "gantry_z.pos": 40.0})
    assert any(line.startswith("MOVE X120.000 Z40.000") for line in fw.sent)
    fw.tick(10)
    obs = hw_robot.get_observation()
    assert obs["gantry_x.pos"] == pytest.approx(120.0, abs=1e-3)
    # a small change below the deadband is not re-sent
    n = len(fw.sent)
    hw_robot.send_action({"gantry_x.pos": 120.2})
    assert not any(l.startswith("MOVE") for l in fw.sent[n:])


def test_ganged_mismatch_faults_and_lights_red(hw_robot, fw):
    hw_robot.bus.pos["elbow_flex_2"] = 9.0  # twin servo slipped / disagrees
    with pytest.raises(CollisionError):
        for _ in range(10):
            hw_robot.get_observation()
    assert any(l.startswith("LED FAULT") for l in fw.sent)
