"""Pack the twin servos of the geared joints inside their pucks (2D, in the plane normal to the joint axis).

    python cad/servo_pack.py  -> cad/out/servo_layout.json   (~2 min, used by cad/internals.py)

Each servo is its STS case outline (45.2 x 24.7, output shaft 11.5 mm from one end). The output shaft
must sit on its pinion's centre (centre distance from params.GEARS). Constraints: inside the puck
(inner wall minus 1.6 mm), the two cases >= 1 mm apart, horns not touching, and at the shoulder the
cases stay above the J1 Oldham/servo stack (z >= -27 relative to J2). Maximises the smallest clearance.
"""
from pathlib import Path
import math, json
from shapely.geometry import Polygon, Point, box as sbox
from shapely import affinity
L, W, S = 45.2, 24.7, 11.5
def body(px, pz, phi):
    r = Polygon([(-S, -W/2), (L-S, -W/2), (L-S, W/2), (-S, W/2)])
    return affinity.translate(affinity.rotate(r, phi, origin=(0, 0)), px, pz)
def solve(R, cdA, cdB, zfloor=None, horn_r=10.0, hub_tip=19.0):
    circ = Point(0, 0).buffer(R, 256)
    best = None
    for a1 in range(0, 360, 5):
        for a2 in range(0, 360, 5):
            if a2 <= a1: continue
            pA = (cdA*math.sin(math.radians(a1)), cdA*math.cos(math.radians(a1)))
            pB = (cdB*math.sin(math.radians(a2)), cdB*math.cos(math.radians(a2)))
            if math.dist(pA, pB) < 2*horn_r + 2: continue      # horns must not touch
            for f1 in range(0, 360, 10):
                b1 = body(*pA, f1)
                if not circ.contains(b1): continue
                if zfloor is not None and b1.bounds[1] < zfloor: continue
                for f2 in range(0, 360, 10):
                    b2 = body(*pB, f2)
                    if not circ.contains(b2): continue
                    if zfloor is not None and b2.bounds[1] < zfloor: continue
                    m = min(b1.distance(b2), circ.exterior.distance(b1), circ.exterior.distance(b2))
                    if best is None or m > best[0]:
                        best = (round(m, 2), dict(A=(round(pA[0],2), round(pA[1],2), f1), B=(round(pB[0],2), round(pB[1],2), f2)))
    return best
j2 = solve(44.0, 27.0, 26.5, zfloor=-27.0)
j3 = solve(40.0, 22.5, 22.0)
print("J2", j2); print("J3", j3)
out = Path(__file__).resolve().parent / "out" / "servo_layout.json"
out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps(dict(J2=j2, J3=j3), indent=1))
print("wrote", out)
