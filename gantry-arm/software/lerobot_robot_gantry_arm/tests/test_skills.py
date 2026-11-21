"""Skills end-to-end in the MuJoCo sim backend (+ synthetic camera images for vision)."""

import logging
import math

import numpy as np
import pytest

pytest.importorskip("mujoco")
cv2 = pytest.importorskip("cv2")
logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)

from lerobot_robot_gantry_arm import GantryArm, GantryArmConfig  # noqa: E402
from lerobot_robot_gantry_arm.kinematics import WORK_AREA, Pose  # noqa: E402
from lerobot_robot_gantry_arm.skills import pick_place, plotter, tool_changer, vision  # noqa: E402
from lerobot_robot_gantry_arm.skills.motion import Motion  # noqa: E402


@pytest.fixture
def motion(tmp_path):
    r = GantryArm(GantryArmConfig(backend="sim", id="s", calibration_dir=tmp_path, max_relative_target=None))
    r.connect()
    m = Motion(r)
    m.move_to(Pose(0, 200, 120, 180))
    m.trace.clear()
    yield m
    r.disconnect()


# ------------------------------------------------------------------ vision


def test_apriltag_calibration_and_object_location():
    objs = {"red_1": (-210.0, 180.0), "blue_1": (160.0, 250.0), "green_1": (20.0, 140.0)}
    img, _ = vision.synthetic_table_image(objs)
    cal = vision.TableCalibration.from_image(img)
    assert cal.rms_mm < 1.5
    found = {d.label: d.xy_mm for d in vision.find_colour_objects(img, cal)}
    for name, xy in objs.items():
        assert math.dist(found[name.split("_")[0]], xy) < 2.5


def test_calibration_follows_the_x_carriage():
    img, _ = vision.synthetic_table_image({"red_1": (0.0, 200.0)})
    cal = vision.TableCalibration.from_image(img, gantry_x=400.0)
    d = vision.find_colour_objects(img, cal, gantry_x=550.0)[0]
    assert d.xy_mm[0] == pytest.approx(150.0, abs=2.5)  # head moved +150 mm with the carriage


def test_calibration_needs_tags():
    with pytest.raises(ValueError):
        vision.TableCalibration.from_image(np.full((300, 400, 3), 200, np.uint8))


# ------------------------------------------------------------------ plotter

SVG = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 200 100">
<rect x="5" y="5" width="190" height="90" fill="none" stroke="black"/>
<circle cx="60" cy="50" r="30" fill="none" stroke="black"/>
<path d="M120 80 L150 20 L180 80 Z" fill="none" stroke="black"/></svg>"""


def test_plan_fits_page_and_splits_wide_strokes():
    strokes = plotter.plan(SVG)
    x0, x1, y0, y1 = plotter.A4_LANDSCAPE
    pts = [p for s in strokes for p in s]
    assert all(x0 <= x <= x1 and y0 <= y <= y1 for x, y in pts)
    assert all(max(x for x, _ in s) - min(x for x, _ in s) <= plotter.PlotParams().max_stroke_span_mm + 1e-6 for s in strokes)


def test_plot_traces_the_drawing_in_sim(motion):
    motion.robot.sim.tool_id = plotter.PEN_TOOL_ID
    strokes = plotter.plan(SVG)
    p = plotter.PlotParams(speed_mm_s=60)
    plotter.plot(motion, strokes, p)
    down = [tr for tr in motion.trace if tr[1][2] < p.pen_down_z + 0.5]
    assert len(down) > 100
    # every commanded pen-down point lies on the planned strokes; the real (sim) tip follows within ~3 mm
    seg_pts = np.array([pt for s in strokes for pt in s])
    for _, cmd, _, actual, _ in down[::7]:
        assert np.min(np.linalg.norm(seg_pts - np.array(cmd[:2]), axis=1)) < 2.0
        assert math.dist(cmd, actual) < 3.0


def test_plot_refuses_without_pen(motion):
    with pytest.raises(RuntimeError):
        plotter.plot(motion, plotter.plan(SVG))


# ------------------------------------------------------------------ pick & place


def test_pick_and_place_sequence_in_sim(motion):
    pick_place.pick_and_place(motion, (-150.0, 180.0), (200.0, 260.0), object_h_mm=30.0)
    closes = [i for i in range(1, len(motion.trace)) if motion.trace[i][2] < motion.trace[i - 1][2]]
    opens = [i for i in range(1, len(motion.trace)) if motion.trace[i][2] > motion.trace[i - 1][2]]
    assert closes and opens
    _, cmd, _, actual, _ = motion.trace[closes[0]]
    assert math.dist(actual[:2], (-150.0, 180.0)) < 3.0 and actual[2] < 25.0  # closed at the object
    _, cmd, _, actual, _ = motion.trace[opens[-1]]
    assert math.dist(actual[:2], (200.0, 260.0)) < 3.0  # released at the target


def test_pick_outside_work_area_rejected(motion):
    with pytest.raises(ValueError):
        pick_place.pick_and_place(motion, (0.0, 600.0), (0.0, 200.0))
    assert WORK_AREA["y"][1] < 600


# ------------------------------------------------------------------ tool changer


def test_tool_change_gripper_to_pen_in_sim(motion):
    rack = tool_changer.Rack.load()
    tool_changer.change_to(motion, 3, rack)
    assert motion.robot.tool_id() == 3
    # the magnet-breaking lift and the mating drop are pure Z-gantry moves: arm joints frozen
    zs = [(i, tr[4]) for i, tr in enumerate(motion.trace)]
    lifts = [q for (i, q), (_, q2) in zip(zs, zs[1:]) if abs(q2["gantry_z"] - q["gantry_z"]) > 1e-6 and abs(q2["shoulder_lift"] - q["shoulder_lift"]) < 1e-9]
    assert len(lifts) > 10
    # the drop really went into the fork: the tool reached the slot centre at docking height
    assert any(math.dist(tr[3], (rack.slots[1], rack.y, rack.tcp_z)) < 3.0 for tr in motion.trace)
    assert any(math.dist(tr[3], (rack.slots[3], rack.y, rack.tcp_z)) < 3.0 for tr in motion.trace)


def test_pick_refuses_when_holding_a_tool(motion):
    with pytest.raises(tool_changer.ToolChangeError):
        tool_changer.pick(motion, 2)
