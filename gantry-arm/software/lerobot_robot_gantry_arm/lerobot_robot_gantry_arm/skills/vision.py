"""Camera -> table calibration with AprilTags, and simple object finding.

Setup: print 4 AprilTags (family 36h11, ids 0-3, 40 mm) and stick them on the table at known
world positions (default: the corners of a rectangle inside WORK_AREA). One snapshot from the
head ("eyes") camera gives a homography from pixels to table millimetres. Because the head rides
on the X carriage, the calibration is stored relative to gantry_x and shifted when it moves.

Objects: coloured blobs (HSV ranges) or AprilTag-labelled parts -> table (x, y) in mm.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

TAG_FAMILY = "DICT_APRILTAG_36h11"
DEFAULT_TAG_WORLD = {  # tag id -> world (x, y) of the tag centre, mm (table plane z = 0)
    0: (-380.0, 130.0),
    1: (380.0, 130.0),
    2: (380.0, 290.0),
    3: (-380.0, 290.0),
}

COLOURS = {  # OpenCV HSV ranges (H 0-179)
    "red": [((0, 120, 80), (8, 255, 255)), ((170, 120, 80), (179, 255, 255))],
    "green": [((40, 80, 60), (85, 255, 255))],
    "blue": [((95, 120, 60), (130, 255, 255))],
    "yellow": [((20, 120, 100), (35, 255, 255))],
}


def _cv():
    import cv2  # optional dependency ([vision] extra)

    return cv2


def detect_tags(image: np.ndarray) -> dict[int, np.ndarray]:
    """Return tag id -> 4x2 corner pixels (OpenCV order: TL, TR, BR, BL)."""
    cv2 = _cv()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, TAG_FAMILY)), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}


@dataclass
class TableCalibration:
    """Pixel -> table-plane homography. `gantry_x_at_cal` lets a carriage-mounted camera move."""

    H: np.ndarray = field(default_factory=lambda: np.eye(3))
    gantry_x_at_cal: float = 0.0
    rms_mm: float = float("nan")

    @classmethod
    def from_image(cls, image: np.ndarray, tag_world: dict | None = None, tag_size_mm: float = 40.0, gantry_x: float = 0.0) -> "TableCalibration":
        cv2 = _cv()
        tag_world = tag_world or DEFAULT_TAG_WORLD
        found = detect_tags(image)
        src, dst = [], []
        h = tag_size_mm / 2
        for tid, (wx, wy) in tag_world.items():
            if tid not in found:
                continue
            # tag corners in world: TL, TR, BR, BL as seen from above (+x right, +y away from the rail)
            world = [(wx - h, wy + h), (wx + h, wy + h), (wx + h, wy - h), (wx - h, wy - h)]
            src.extend(found[tid].tolist())
            dst.extend(world)
        if len(src) < 8:
            raise ValueError(f"need at least 2 calibration tags in view, found {sorted(found)}")
        H, _ = cv2.findHomography(np.float32(src), np.float32(dst), 0)
        cal = cls(H=H, gantry_x_at_cal=gantry_x)
        err = np.linalg.norm(cal.to_table(np.float32(src)) - np.float32(dst), axis=1)
        cal.rms_mm = float(np.sqrt(np.mean(err**2)))
        return cal

    def to_table(self, px: np.ndarray, gantry_x: float | None = None) -> np.ndarray:
        pts = np.asarray(px, dtype=np.float64).reshape(-1, 2)
        homo = np.hstack([pts, np.ones((len(pts), 1))]) @ self.H.T
        xy = homo[:, :2] / homo[:, 2:3]
        if gantry_x is not None:  # camera rides on the X carriage
            xy[:, 0] += gantry_x - self.gantry_x_at_cal
        return xy

    def save(self, path: Path):
        path.write_text(json.dumps({"H": self.H.tolist(), "gantry_x_at_cal": self.gantry_x_at_cal, "rms_mm": self.rms_mm}, indent=1))

    @classmethod
    def load(cls, path: Path) -> "TableCalibration":
        d = json.loads(path.read_text())
        return cls(H=np.array(d["H"]), gantry_x_at_cal=d["gantry_x_at_cal"], rms_mm=d["rms_mm"])


@dataclass
class Detection:
    label: str
    px: tuple[float, float]
    xy_mm: tuple[float, float]
    area_px: float


def find_colour_objects(image: np.ndarray, cal: TableCalibration, colours=("red", "green", "blue", "yellow"), min_area_px: float = 150.0, gantry_x: float | None = None) -> list[Detection]:
    cv2 = _cv()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    out = []
    for name in colours:
        mask = None
        for lo, hi in COLOURS[name]:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else mask | m
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask)
        for i in range(1, n):
            area = float(stats[i, cv2.CC_STAT_AREA])
            if area < min_area_px:
                continue
            c = tuple(map(float, cents[i]))
            xy = cal.to_table(np.array([c]), gantry_x)[0]
            out.append(Detection(name, c, (float(xy[0]), float(xy[1])), area))
    return sorted(out, key=lambda d: -d.area_px)


def find_tagged_objects(image: np.ndarray, cal: TableCalibration, ignore_ids=tuple(DEFAULT_TAG_WORLD), gantry_x: float | None = None) -> list[Detection]:
    out = []
    for tid, corners in detect_tags(image).items():
        if tid in ignore_ids:
            continue
        c = corners.mean(axis=0)
        xy = cal.to_table(c[None], gantry_x)[0]
        out.append(Detection(f"tag{tid}", (float(c[0]), float(c[1])), (float(xy[0]), float(xy[1])), 0.0))
    return out


def synthetic_table_image(objects: dict | None = None, tag_world: dict | None = None, tag_size_mm: float = 40.0, px_per_mm: float = 1.0,
                          cam_tilt: float = 0.15, size=(1100, 520)) -> tuple[np.ndarray, np.ndarray]:
    """A perspective photo of the table with tags and coloured discs, plus its true world->pixel
    homography. Used for tests and the demo (no camera needed)."""
    cv2 = _cv()
    tag_world = tag_world or DEFAULT_TAG_WORLD
    W, Hh = 1000, 260  # canvas covering world x -500..500, y 80..340 at 1 px/mm
    canvas = np.full((Hh, W, 3), 225, np.uint8)

    def w2c(x, y):
        return int(round(x + 500)), int(round(340 - y))

    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, TAG_FAMILY))
    s = int(tag_size_mm)
    for tid, (wx, wy) in tag_world.items():
        tag = cv2.aruco.generateImageMarker(d, tid, s)
        cx, cy = w2c(wx, wy)
        pad = np.full((s + 16, s + 16), 255, np.uint8)
        pad[8:-8, 8:-8] = tag
        canvas[cy - s // 2 - 8: cy - s // 2 - 8 + s + 16, cx - s // 2 - 8: cx - s // 2 - 8 + s + 16] = cv2.cvtColor(pad, cv2.COLOR_GRAY2BGR)
    bgr = {"red": (40, 40, 210), "green": (60, 170, 60), "blue": (200, 90, 30), "yellow": (40, 210, 230)}
    for name, (x, y) in (objects or {}).items():
        cv2.circle(canvas, w2c(x, y), 14, bgr[name.split("_")[0]], -1)
    # canvas (world mm) -> image pixels: a gentle perspective, as from the column-top head
    src = np.float32([[0, 0], [W, 0], [W, Hh], [0, Hh]])
    k = cam_tilt
    dst = np.float32([[size[0] * k, 30], [size[0] * (1 - k), 30], [size[0] - 20, size[1] - 20], [20, size[1] - 20]])
    P = cv2.getPerspectiveTransform(src, dst)
    img = cv2.warpPerspective(canvas, P, size, borderValue=(200, 200, 200))
    C = np.array([[1, 0, 500], [0, -1, 340], [0, 0, 1]], float)  # world -> canvas
    return img, P @ C
