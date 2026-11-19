"""Pick-and-place with the parallel gripper (tool 1), tool pointing straight down.

    approach above -> descend -> close -> lift -> carry -> descend -> open -> retreat

Heights are relative to the table (z = 0). Objects come from vision.find_* (x, y in mm).
"""

from __future__ import annotations

from dataclasses import dataclass

from ..kinematics import WORK_AREA, Pose
from .motion import Motion

GRIPPER_TOOL_ID = 1


@dataclass
class PickPlaceParams:
    approach_mm: float = 60.0  # clearance above the object top before descending
    grasp_depth_mm: float = 12.0  # how far below the object top the finger pads close
    open_pct: float = 100.0
    close_pct: float = 10.0
    descend_mm_s: float = 40.0


def in_work_area(x: float, y: float) -> bool:
    (x0, x1), (y0, y1) = WORK_AREA["x"], WORK_AREA["y"]
    return x0 <= x <= x1 and y0 <= y <= y1


def pick_and_place(m: Motion, pick_xy, place_xy, object_h_mm: float = 30.0, yaw_deg: float = 0.0, p: PickPlaceParams | None = None):
    p = p or PickPlaceParams()
    for xy in (pick_xy, place_xy):
        if not in_work_area(*xy):
            raise ValueError(f"{xy} is outside the table work area {WORK_AREA}")
    tool = getattr(m.robot, "tool_id", lambda: GRIPPER_TOOL_ID)()
    if tool != GRIPPER_TOOL_ID:
        raise RuntimeError(f"gripper tool required, tool id {tool} is mounted (use tool_changer.change_to(1))")
    grasp_z = max(object_h_mm - p.grasp_depth_mm, 4.0)
    high = object_h_mm + p.approach_mm

    m.set_gripper(p.open_pct, wait_s=0.3)
    m.move_to(Pose(*pick_xy, high, 180.0, yaw_deg), also=[Pose(*pick_xy, grasp_z, 180.0, yaw_deg)])
    m.line_to(Pose(*pick_xy, grasp_z, 180.0, yaw_deg), speed_mm_s=p.descend_mm_s)
    m.set_gripper(p.close_pct)
    m.line_to(Pose(*pick_xy, high, 180.0, yaw_deg), speed_mm_s=p.descend_mm_s)
    m.move_to(Pose(*place_xy, high, 180.0, yaw_deg), also=[Pose(*place_xy, grasp_z + 2.0, 180.0, yaw_deg)])
    m.line_to(Pose(*place_xy, grasp_z + 2.0, 180.0, yaw_deg), speed_mm_s=p.descend_mm_s)
    m.set_gripper(p.open_pct)
    m.line_to(Pose(*place_xy, high, 180.0, yaw_deg), speed_mm_s=p.descend_mm_s)


def sort_by_colour(m: Motion, detections, bins: dict, object_h_mm: float = 25.0):
    """Move every detected object onto its colour's bin position, e.g. bins={"red": (300, 280)}."""
    done = []
    for d in detections:
        if d.label in bins and in_work_area(*d.xy_mm):
            pick_and_place(m, d.xy_mm, bins[d.label], object_h_mm)
            done.append(d.label)
    return done
