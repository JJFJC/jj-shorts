import json
from pathlib import Path

import pytest

from lerobot_robot_gantry_arm.dynamics import CollisionMonitor, Ganged, decode_load, feedforward, servo_kp_nm_per_deg

DESIGN = Path(__file__).resolve().parents[3]


def test_feedforward_offsets_goal_by_tau_over_kp_and_clamps():
    out = feedforward({"a": 10.0, "b": 0.0}, {"a": 1.0, "b": -100.0}, {"a": 0.5, "b": 1.0}, max_offset_deg=8)
    assert out["a"] == pytest.approx(12.0)
    assert out["b"] == pytest.approx(-8.0)


def test_kp_scales_with_p_coefficient_and_servo_count():
    k1 = servo_kp_nm_per_deg(2.942, 1, 16)
    assert servo_kp_nm_per_deg(2.942, 2, 32) == pytest.approx(4 * k1)


def test_ganged_expand_merge_and_mismatch_watchdog():
    g = Ganged(tolerance_deg=3, cycles=2)
    assert g.expand({"shoulder_lift": 40.0})["shoulder_lift_2"] == 40.0
    assert g.merge({"shoulder_lift": 40.0, "shoulder_lift_2": 42.0}) == {"shoulder_lift": 41.0}
    ok = {"shoulder_lift": 40.0, "shoulder_lift_2": 41.0, "elbow_flex": 0.0, "elbow_flex_2": 0.0}
    bad = {**ok, "elbow_flex_2": 6.0}
    assert g.check(ok) == []
    assert g.check(bad) == []  # one bad read is tolerated
    assert g.check(bad) == ["elbow_flex"]


def test_decode_load_sign_bit():
    assert decode_load(250) == pytest.approx(25.0)
    assert decode_load(0x400 | 250) == pytest.approx(-25.0)
    assert decode_load(-120) == pytest.approx(-12.0)


def test_collision_needs_load_error_and_stall_together():
    m = CollisionMonitor(threshold_pct=20, cycles=2, min_error_deg=3, max_speed_deg_s=4)
    exp = {"elbow_flex": 10.0}
    # accelerating toward a goal: high load + error, but moving -> not a collision
    for _ in range(5):
        assert m.update(exp, {"elbow_flex": 60.0}, {"elbow_flex": 10.0}, {"elbow_flex": 40.0}) is None
    # heavy payload: load explained by gravity model -> not a collision
    for _ in range(5):
        assert m.update({"elbow_flex": 55.0}, {"elbow_flex": 60.0}, {"elbow_flex": 10.0}, {"elbow_flex": 0.0}) is None
    # blocked: load not explained, error persists, joint stopped
    assert m.update(exp, {"elbow_flex": 60.0}, {"elbow_flex": 10.0}, {"elbow_flex": 0.5}) is None
    assert m.update(exp, {"elbow_flex": 60.0}, {"elbow_flex": 10.0}, {"elbow_flex": 0.5}) == "elbow_flex"


def test_gravity_model_matches_design_analysis():
    pytest.importorskip("mujoco")
    res = DESIGN / "sim" / "out" / "results.json"
    if not res.exists():
        pytest.skip("design analysis not present")
    from lerobot_robot_gantry_arm.dynamics import GravityModel

    want = json.loads(res.read_text())["spring"]["full_reach"]
    got = GravityModel(payload_g=500).torques({"shoulder_lift": 90, "elbow_flex": 0, "wrist_flex": 0, "wrist_roll": 0, "base_yaw": 0})
    for j in ("shoulder_lift", "elbow_flex", "wrist_flex"):
        assert got[j] == pytest.approx(want[j], abs=1e-6)
