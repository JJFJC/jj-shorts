"""`gantry-arm` command line: run skills on the real robot or (with --sim) on the MuJoCo model.

    gantry-arm plot drawing.svg --sim
    gantry-arm tool 3                      # automatic tool change to the pen
    gantry-arm calibrate-camera --camera 0 --out table_cal.json
    gantry-arm sort --camera 0 --cal table_cal.json --bins red=330,160 blue=330,220
"""

from __future__ import annotations

import argparse
import logging
import tempfile
from pathlib import Path

from . import GantryArm, GantryArmConfig
from .kinematics import Pose
from .skills import pick_place, plotter, tool_changer, vision
from .skills.motion import Motion


def _robot(a) -> GantryArm:
    cfg = GantryArmConfig(
        backend="sim" if a.sim else "hardware", port=a.port, gantry_port=a.gantry_port, id=a.id,
        calibration_dir=Path(a.calibration_dir) if a.calibration_dir else Path(tempfile.mkdtemp()) if a.sim else None,
    )
    r = GantryArm(cfg)
    r.connect()
    return r


def _grab(camera: int):
    import cv2

    cap = cv2.VideoCapture(camera)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    ok, img = cap.read()
    cap.release()
    if not ok:
        raise SystemExit(f"camera {camera}: no frame")
    return img


def main(argv=None):
    ap = argparse.ArgumentParser("gantry-arm")
    ap.add_argument("--sim", action="store_true", help="run on the MuJoCo model instead of hardware")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--gantry-port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="gantry_arm")
    ap.add_argument("--calibration-dir", default=None)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plot", help="draw an SVG with the pen tool")
    p.add_argument("svg")
    p.add_argument("--speed", type=float, default=40.0)
    t = sub.add_parser("tool", help="change tool: 1 gripper, 2 suction, 3 pen, 4 phone")
    t.add_argument("tool_id", type=int)
    c = sub.add_parser("calibrate-camera", help="AprilTag table calibration from the eyes camera")
    c.add_argument("--camera", type=int, default=0)
    c.add_argument("--out", default="table_cal.json")
    s = sub.add_parser("sort", help="pick coloured objects and drop them on their bins")
    s.add_argument("--camera", type=int, default=0)
    s.add_argument("--cal", default="table_cal.json")
    s.add_argument("--bins", nargs="+", required=True, help="colour=x,y in table mm, e.g. red=330,160")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO)

    if a.cmd == "calibrate-camera":
        img = _grab(a.camera)
        cal = vision.TableCalibration.from_image(img)
        cal.save(Path(a.out))
        print(f"saved {a.out}: {cal.rms_mm:.2f} mm RMS")
        return

    robot = _robot(a)
    m = Motion(robot)
    try:
        if a.cmd == "plot":
            if a.sim:
                robot.sim.tool_id = plotter.PEN_TOOL_ID
            plotter.plot(m, plotter.plan(Path(a.svg).read_text()), plotter.PlotParams(speed_mm_s=a.speed))
        elif a.cmd == "tool":
            tool_changer.change_to(m, a.tool_id)
        elif a.cmd == "sort":
            bins = {k: tuple(float(v) for v in xy.split(",")) for k, xy in (b.split("=") for b in a.bins)}
            if a.sim:
                img, _ = vision.synthetic_table_image({"red_1": (-250.0, 180.0), "blue_1": (0.0, 240.0)})
                cal = vision.TableCalibration.from_image(img)
            else:
                img, cal = _grab(a.camera), vision.TableCalibration.load(Path(a.cal))
            gx = m.joints()["gantry_x"]
            dets = vision.find_colour_objects(img, cal, colours=tuple(bins), gantry_x=None if a.sim else gx)
            print("sorted:", pick_place.sort_by_colour(m, dets, bins))
        m.move_to(Pose(0, 200, 150, 180))
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
