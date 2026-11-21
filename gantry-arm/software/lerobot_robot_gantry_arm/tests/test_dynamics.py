import json
from pathlib import Path

import pytest

from lerobot_robot_gantry_arm.dynamics import CollisionMonitor, Ganged, GearedPair, decode_load, feedforward, servo_kp_nm_per_deg

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


# ------------------------------------------------------------------ geared vernier pairs (docs/MECHANICS.md)


def _wrap(x):
    return (x + 180.0) % 360.0 - 180.0


def test_geared_expand_merge_round_trip_with_preload():
    g = GearedPair()
    m = g.expand({"shoulder_lift": 47.5, "elbow_flex": -120.0})
    assert m["shoulder_lift"] == pytest.approx(2.0 * 47.5 + 0.4)
    assert m["shoulder_lift_2"] == pytest.approx(36 / 17 * 47.5 - 0.4)
    back = g.merge(m)  # the +/- preload averages out (to within preload/ratio)
    assert back["shoulder_lift"] == pytest.approx(47.5, abs=0.2)
    assert back["elbow_flex"] == pytest.approx(-120.0, abs=0.2)


@pytest.mark.parametrize("joint,lo,hi", [("shoulder_lift", -100.0, 135.0), ("elbow_flex", -140.0, 140.0)])
def test_vernier_recovers_the_absolute_joint_angle_over_the_whole_range(joint, lo, hi):
    g = GearedPair()
    ra, rb = g.ratios[joint]
    import random

    rnd = random.Random(1)
    q = lo
    while q <= hi:
        noise_a, noise_b = rnd.uniform(-0.6, 0.6), rnd.uniform(-0.6, 0.6)  # backlash + encoder, motor deg
        a, b = _wrap(ra * q + noise_a), _wrap(rb * q + noise_b)
        assert g.absolute(joint, a, b, lo, hi) == pytest.approx(q, abs=0.5)
        q += 0.7


def test_vernier_refuses_readings_that_fit_no_joint_angle():
    g = GearedPair()
    ra, rb = g.ratios["shoulder_lift"]
    a, b = _wrap(ra * 30.0), _wrap(rb * 30.0 + 10.6)  # B off by half the candidate spacing
    with pytest.raises(ValueError):
        g.absolute("shoulder_lift", a, b, -100.0, 135.0)


def test_unwrap_counts_turns_after_seeding():
    g = GearedPair()
    ra, rb = g.ratios["elbow_flex"]
    q = 100.0
    a, b = _wrap(ra * q), _wrap(rb * q)
    g.seed("elbow_flex", g.absolute("elbow_flex", a, b, -140, 140), a, b)
    out = {}
    for q in (100.0, 110.0, 120.0, 130.0, 120.0, 95.0):  # crosses the +/-180 wrap on both servos
        out = g.unwrap({"elbow_flex": _wrap(ra * q), "elbow_flex_2": _wrap(rb * q)})
        assert g.merge(out)["elbow_flex"] == pytest.approx(q, abs=1e-6)


def test_geared_check_flags_a_slipped_pinion():
    g = GearedPair(cycles=2)
    ok = {"shoulder_lift": 2.0 * 20, "shoulder_lift_2": 36 / 17 * 20}
    bad = {**ok, "shoulder_lift_2": 36 / 17 * 20 + 36 / 17 * 4}  # one servo 4 deg (joint) off
    assert g.check(ok) == []
    assert g.check(bad) == []
    assert g.check(bad) == ["shoulder_lift"]
