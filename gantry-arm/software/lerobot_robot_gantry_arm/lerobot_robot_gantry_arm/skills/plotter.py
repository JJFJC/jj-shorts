"""Pen plotting from SVG with the pen-holder tool (tool 3).

The drawing is scaled into a page rectangle inside WORK_AREA. Strokes wider than the arm can
cover from one gantry position are split; the gantry repositions only while the pen is up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..kinematics import WORK_AREA, IKError, Pose
from .motion import Motion

PEN_TOOL_ID = 3
A4_LANDSCAPE = (-148.5, 297 - 148.5, 110.0, 110.0 + 190.0)  # x0, x1, y0, y1 (page fits the 190 mm band)


@dataclass
class PlotParams:
    pen_down_z: float = 2.0  # tool tip height when drawing (holder spring takes up +-3 mm)
    pen_up_z: float = 15.0
    speed_mm_s: float = 40.0
    max_stroke_span_mm: float = 260.0  # x-span drawable without moving the gantry
    tolerance_mm: float = 0.5  # curve flattening


def svg_to_polylines(svg_text: str, tolerance_mm: float = 0.5) -> list[list[tuple[float, float]]]:
    """Every path / basic shape -> list of polylines in SVG user units (y down)."""
    from svgpathtools import svgstr2paths

    paths, _ = svgstr2paths(svg_text)
    out = []
    for path in paths:
        for sub in path.continuous_subpaths():
            if sub.length() == 0:
                continue
            n = max(2, int(math.ceil(sub.length() / tolerance_mm)))
            pts = [sub.point(i / (n - 1)) for i in range(n)]
            out.append([(p.real, p.imag) for p in pts])
    return out


def fit_to_page(lines, page=A4_LANDSCAPE, margin_mm: float = 10.0):
    """Uniformly scale + centre polylines into the page; flips SVG's y-down to table y-up
    (the drawing's top edge ends up farthest from the rail)."""
    xs = [x for ln in lines for x, _ in ln]
    ys = [y for ln in lines for _, y in ln]
    x0, x1, y0, y1 = page
    w, h = (x1 - x0) - 2 * margin_mm, (y1 - y0) - 2 * margin_mm
    sx = w / max(1e-9, max(xs) - min(xs))
    sy = h / max(1e-9, max(ys) - min(ys))
    s = min(sx, sy)
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    px, py = (x0 + x1) / 2, (y0 + y1) / 2
    return [[(px + (x - cx) * s, py - (y - cy) * s) for x, y in ln] for ln in lines]


def split_wide(lines, max_span: float):
    out = []
    for ln in lines:
        cur = [ln[0]]
        for pt in ln[1:]:
            xs = [p[0] for p in cur] + [pt[0]]
            if max(xs) - min(xs) > max_span:
                out.append(cur)
                cur = [cur[-1], pt]
            else:
                cur.append(pt)
        out.append(cur)
    return out


def order_strokes(lines):
    """Greedy nearest-neighbour ordering (also reverses strokes) to cut pen-up travel."""
    left = list(lines)
    out, pos = [], (A4_LANDSCAPE[0], A4_LANDSCAPE[2])
    while left:
        best = min(range(len(left)), key=lambda i: min(math.dist(pos, left[i][0]), math.dist(pos, left[i][-1])))
        ln = left.pop(best)
        if math.dist(pos, ln[-1]) < math.dist(pos, ln[0]):
            ln = ln[::-1]
        out.append(ln)
        pos = ln[-1]
    return out


def plan(svg_text: str, page=A4_LANDSCAPE, p: PlotParams | None = None):
    p = p or PlotParams()
    lines = fit_to_page(svg_to_polylines(svg_text, p.tolerance_mm), page)
    return order_strokes(split_wide(lines, p.max_stroke_span_mm))


def _stroke_gantry(m: Motion, ln, p: PlotParams, prefer=None):
    """Gantry (x, z) that reaches the whole stroke at pen-down and pen-up height (sampled)."""
    step = max(1, len(ln) // 25)
    sample = ln[::step] + [ln[-1]]
    poses = [Pose(x, y, p.pen_down_z, 180.0) for x, y in sample] + [Pose(ln[0][0], ln[0][1], p.pen_up_z, 180.0), Pose(ln[-1][0], ln[-1][1], p.pen_up_z, 180.0)]
    return m.kin.common_gantry(poses, prefer=prefer)


def _plan_gantry(m: Motion, strokes, p: PlotParams):
    """Attach a gantry position to every stroke, halving strokes that no single position covers."""
    out, todo = [], list(strokes)
    while todo:
        ln = todo.pop(0)
        try:
            out.append((ln, _stroke_gantry(m, ln, p)))
        except IKError:
            if len(ln) < 4:
                raise
            mid = len(ln) // 2
            todo[0:0] = [ln[: mid + 1], ln[mid:]]
    return out


def plot(m: Motion, strokes, p: PlotParams | None = None, check_tool: bool = True):
    p = p or PlotParams()
    if check_tool:
        tool = getattr(m.robot, "tool_id", lambda: PEN_TOOL_ID)()
        if tool != PEN_TOOL_ID:
            raise RuntimeError(f"pen tool required, tool id {tool} is mounted")
    for ln, (gx, gz) in _plan_gantry(m, strokes, p):
        sx, sy = ln[0]
        ex, ey = ln[-1]
        # pen up: travel (gantry may move), then everything in the stroke is pure arm motion
        m.move_to(Pose(sx, sy, p.pen_up_z, 180.0), gantry_x=gx, gantry_z=gz)
        m.line_to(Pose(sx, sy, p.pen_down_z, 180.0), speed_mm_s=20.0)
        m.path(ln, p.pen_down_z, speed_mm_s=p.speed_mm_s)  # IKError propagates: never skip ink silently
        m.line_to(Pose(ex, ey, p.pen_up_z, 180.0), speed_mm_s=20.0)
