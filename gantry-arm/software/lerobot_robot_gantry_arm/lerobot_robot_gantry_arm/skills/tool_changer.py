"""Automatic tool change from the passive rack (cad: tool_rack_1/2, params.RACK).

Nothing on the rack moves. The coupler's bayonet sleeve is spring-loaded to "locked"; its lever
tab drops into a cam channel on the rack post, which holds the sleeve open while docked.

DROP (tool in hand -> slot):
    above the slot front, wrist rolled so the lever faces the cam post
    -> slide +Y into the fork (fork enters the tool's parking groove; lever enters the cam)
    -> Z gantry lifts `lift_mm` (fork holds the tool, magnets let go; arm joints don't move)
    -> back off -Y
PICK (empty -> slot):
    above the slot, raised by `lift_mm` -> Z gantry lowers (cone aligns, magnets snap)
    -> slide -Y out of the fork (lever leaves the cam, sleeve springs locked)
    -> check TOOL_ID on the pogo pin matches

The Z gantry (T8x2 lead screw, ~300 N available) does the magnet-breaking, not the servos.
"""

from __future__ import annotations

import json
from dataclasses import dataclass

from ..kinematics import ASSETS, Pose
from .motion import Motion

TOOL_NAMES = {0: "none", 1: "gripper", 2: "suction", 3: "pen", 4: "phone mount"}


@dataclass(frozen=True)
class Rack:
    slots: dict
    y: float
    tcp_z: float
    insert_mm: float
    lift_mm: float
    roll_deg: float

    @classmethod
    def load(cls) -> "Rack":
        r = json.loads((ASSETS / "geometry.json").read_text())["rack"]
        return cls({int(k): v for k, v in r["slots"].items()}, r["y"], r["tcp_z"], r["insert_mm"], r["lift_mm"], r["roll_deg"])


class ToolChangeError(RuntimeError):
    pass


def _set_tool(m: Motion, tool_id: int):
    sim = getattr(m.robot, "sim", None)
    if sim is not None:  # the sim has no pogo pins: emulate what TOOL_ID would read
        sim.tool_id = tool_id


def _mounted(m: Motion) -> int:
    return getattr(m.robot, "tool_id", lambda: 0)()


def _dock_gantry(m: Motion, x: float, rack: Rack) -> tuple[float, float]:
    """One gantry (x, z) for the whole docking sequence, expressed with the gantry in its RAISED
    position. Poses visited while it is `lift_mm` lower are shifted up by lift_mm (same arm pose)."""
    r = rack.roll_deg
    front, z, L = rack.y - rack.insert_mm, rack.tcp_z, rack.lift_mm
    raised = [Pose(x, front, z + L + 30, 180, r), Pose(x, rack.y, z + L, 180, r), Pose(x, rack.y, z + L + 20, 180, r)]
    lowered = [Pose(x, front, z, 180, r), Pose(x, rack.y, z, 180, r), Pose(x, front, z + 40, 180, r)]
    return m.kin.common_gantry(raised + [Pose(p.x, p.y, p.z + L, p.tilt, p.roll) for p in lowered], prefer=m.joints(), z_min=L)


def drop(m: Motion, rack: Rack | None = None):
    rack = rack or Rack.load()
    tid = _mounted(m)
    if tid not in rack.slots:
        raise ToolChangeError(f"mounted tool {tid} has no rack slot")
    x = rack.slots[tid]
    gx, gz = _dock_gantry(m, x, rack)
    front = rack.y - rack.insert_mm
    m.move_to(Pose(x, front, rack.tcp_z + 30.0, 180.0, rack.roll_deg), gantry_x=gx, gantry_z=gz - rack.lift_mm)
    m.line_to(Pose(x, front, rack.tcp_z, 180.0, rack.roll_deg), speed_mm_s=25.0)
    m.line_to(Pose(x, rack.y, rack.tcp_z, 180.0, rack.roll_deg), speed_mm_s=20.0)
    q = m.joints()
    m.move_joints({"gantry_z": q["gantry_z"] + rack.lift_mm})  # only Z moves: fork holds the tool
    _set_tool(m, 0)
    if _mounted(m) != 0:
        raise ToolChangeError(f"tool {tid} still reads as mounted after drop")


def pick(m: Motion, tool_id: int, rack: Rack | None = None):
    rack = rack or Rack.load()
    if _mounted(m) != 0:
        raise ToolChangeError("drop the current tool first")
    if tool_id not in rack.slots:
        raise ToolChangeError(f"no rack slot for tool {tool_id}")
    x = rack.slots[tool_id]
    gx, gz = _dock_gantry(m, x, rack)
    m.move_to(Pose(x, rack.y, rack.tcp_z + rack.lift_mm + 20.0, 180.0, rack.roll_deg), gantry_x=gx, gantry_z=gz)
    m.line_to(Pose(x, rack.y, rack.tcp_z + rack.lift_mm, 180.0, rack.roll_deg), speed_mm_s=20.0)
    q = m.joints()
    m.move_joints({"gantry_z": q["gantry_z"] - rack.lift_mm})  # mate: cone aligns, magnets snap
    _set_tool(m, tool_id)
    m.line_to(Pose(x, rack.y - rack.insert_mm, rack.tcp_z, 180.0, rack.roll_deg), speed_mm_s=20.0)
    m.line_to(Pose(x, rack.y - rack.insert_mm, rack.tcp_z + 40.0, 180.0, rack.roll_deg), speed_mm_s=30.0)
    got = _mounted(m)
    if got != tool_id:
        raise ToolChangeError(f"expected {TOOL_NAMES.get(tool_id)} (id {tool_id}), TOOL_ID reads {got}")


def change_to(m: Motion, tool_id: int, rack: Rack | None = None):
    rack = rack or Rack.load()
    cur = _mounted(m)
    if cur == tool_id:
        return
    if cur != 0:
        drop(m, rack)
    pick(m, tool_id, rack)
