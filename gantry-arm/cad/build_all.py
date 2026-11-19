"""Build every part, export STEP + STL to cad/out/, write mass properties for the sim,
check every printed part fits the Bambu P2S bed, and write docs/PRINT_LIST.md.

    python cad/build_all.py            # parts + assembly (home pose)
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cadquery as cq
from cadquery import Vector as V

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(HERE), str(ROOT), str(ROOT / "sim")]
import params as P  # noqa: E402
import parts as C  # noqa: E402

import os

OUT = Path(os.environ.get("GANTRY_CAD_OUT", HERE / "out"))
ARM_BODIES = ("z_carriage", "turret", "upper_arm", "forearm", "wrist", "tool")

MAT_DENSITY = {
    "PETG": "PETG", "metal": "steel", "brass": "brass", "rubber": "rubber", "lens": "glass",
    "extrusion": "aluminium", "strip": "PETG", "belt": "rubber", "estop_red": "PETG", "estop_yellow": "PETG",
}


def solid(p: C.Part):
    return p.shape.val() if p.shape.size() == 1 else p.shape.combine().val()


def density(p: C.Part) -> float:
    if p.printed:
        return P.DENSITY["PETG"]
    if "magnet" in p.name:
        return P.DENSITY["magnet"]
    return P.DENSITY[MAT_DENSITY.get(p.role, "PETG")]


def fits_bed(dims) -> tuple[bool, str]:
    """Try every in-plane rotation for each choice of 'up' axis."""
    bx, by, bz = (b - 2 * P.BED_MARGIN for b in P.BED)
    best = None
    for up in range(3):
        h = dims[up]
        a, b = [d for i, d in enumerate(dims) if i != up]
        if h > bz:
            continue
        for deg in range(0, 91):
            t = math.radians(deg)
            w = a * math.cos(t) + b * math.sin(t)
            d = a * math.sin(t) + b * math.cos(t)
            if w <= bx and d <= by:
                note = "flat" if deg == 0 else f"rotated {deg}° on bed"
                return True, note
        best = "too large"
    return False, best or "too tall"


def print_time_h(mass_g: float) -> float:
    """Rough P2S PETG estimate (0.2 mm layers, 6 walls): ~18 g/h + 10 min warm-up/purge."""
    return mass_g / 18.0 + 0.17


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "step").mkdir(exist_ok=True)
    (OUT / "stl").mkdir(exist_ok=True)
    parts = C.build_all()
    manifest = []
    props = {b: {"m": 0.0, "mc": V(0, 0, 0), "bb": None} for b in ARM_BODIES}
    for p in parts:
        s = solid(p)
        vol = s.Volume()
        com = cq.Shape.centerOfMass(s)
        m = vol * density(p) * p.fill  # non-printed parts use fill for hollow profiles (extrusions)
        bb = s.BoundingBox()
        dims = sorted([bb.xlen, bb.ylen, bb.zlen], reverse=True)
        ok, how = fits_bed([bb.xlen, bb.ylen, bb.zlen]) if p.printed else (True, "bought")
        cq.exporters.export(p.shape, str(OUT / "step" / f"{p.name}.step"))
        cq.exporters.export(p.shape, str(OUT / "stl" / f"{p.name}.stl"), tolerance=0.03, angularTolerance=0.08)
        manifest.append(dict(
            name=p.name, body=p.body, role=p.role, qty=p.qty, printed=p.printed, material=p.material,
            orient=p.orient, mass_g=round(m, 1), dims_mm=[round(d, 1) for d in dims], fits_bed=ok, bed_note=how,
            print_h=round(print_time_h(m), 2) if p.printed else 0, group=p.group, note=p.extra.get("note", ""),
        ))
        if p.body in props:
            pr = props[p.body]
            pr["m"] += m
            pr["mc"] = pr["mc"] + com * m
            pr["bb"] = bb if pr["bb"] is None else pr["bb"].add(bb)
    mass_props = {}
    for b, pr in props.items():
        hw = P.HARDWARE_G.get(b, 0.0)
        c = pr["mc"] / pr["m"]
        bb = pr["bb"]
        mass_props[b] = dict(mass_g=round(pr["m"] + hw, 1), printed_g=round(pr["m"], 1), hardware_g=hw,
                             com=[round(c.x, 2), round(c.y, 2), round(c.z, 2)], size=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)])
    (OUT / "mass_props.json").write_text(json.dumps(mass_props, indent=1))
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=1))
    assembly(parts)
    if "GANTRY_CAD_OUT" not in os.environ:  # concept builds never overwrite the real print list
        write_print_list(manifest, mass_props)
    bad = [m["name"] for m in manifest if not m["fits_bed"]]
    print(f"{len(manifest)} parts, {sum(m['printed'] for m in manifest)} printed; bed misfits: {bad or 'none'}")
    print(json.dumps(mass_props, indent=1))


ROLE_COLOR = {
    "shell": (0.22, 0.23, 0.25), "cap": (0.45, 0.46, 0.48), "accent": (0.2, 0.75, 1.0), "extrusion": (0.08, 0.08, 0.09),
    "strip": (0.15, 0.15, 0.16), "metal": (0.75, 0.76, 0.78), "rubber": (0.05, 0.05, 0.05), "smoke": (0.2, 0.2, 0.22),
    "lens": (0.02, 0.02, 0.05), "brass": (0.8, 0.65, 0.3), "estop_red": (0.8, 0.05, 0.05), "estop_yellow": (0.95, 0.8, 0.0),
    "belt": (0.05, 0.05, 0.05),
}

POSE_RENDER = dict(gantry_x=430.0, gantry_z=230.0, base_yaw=-25.0, shoulder_lift=38.0, elbow_flex=62.0, wrist_flex=58.0, wrist_roll=0.0)


def assembly(parts, q=None):
    import fk

    poses = fk.body_poses(q or POSE_RENDER)
    asm = cq.Assembly(name="gantry_arm")
    for p in parts:
        if p.body not in poses:
            continue  # spare tools are not mounted
        pos, quat = poses[p.body]
        w, x, y, z = quat
        ang = 2 * math.degrees(math.acos(max(-1.0, min(1.0, w))))
        s = math.sqrt(max(1e-12, 1 - w * w))
        axis = (x / s, y / s, z / s) if s > 1e-6 else (0, 0, 1)
        loc = cq.Location(V(*pos), V(*axis), ang)
        asm.add(p.shape, name=p.name, loc=loc, color=cq.Color(*ROLE_COLOR.get(p.role, (0.5, 0.5, 0.5))))
    asm.save(str(OUT / "gantry_arm_assembly.step"))


def write_print_list(manifest, mass_props):
    pr = [m for m in manifest if m["printed"]]
    lines = ["# Gantry Arm — print list", "",
             "Generated by `cad/build_all.py`. Printer: Bambu Lab P2S (256 × 256 × 256 mm). Every part was checked against the bed with a 6 mm margin.",
             "Times are rough (≈18 g/h PETG, 0.20 mm layers, 6 walls). Re-slice in Bambu Studio for the real numbers.", "",
             "| # | Part | Qty | Material | Mass each (g) | Est. time each (h) | Size (mm) | Bed | Orientation | Notes |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    tot_m = tot_h = 0.0
    for i, m in enumerate(sorted(pr, key=lambda m: (m["group"], m["name"])), 1):
        tot_m += m["mass_g"] * m["qty"]
        tot_h += m["print_h"] * m["qty"]
        lines.append(f"| {i} | `{m['name']}` | {m['qty']} | {m['material']} | {m['mass_g']:.0f} | {m['print_h']:.1f} | {' × '.join(f'{d:.0f}' for d in m['dims_mm'])} | "
                     f"{'✅ ' + m['bed_note'] if m['fits_bed'] else '❌ ' + m['bed_note']} | {m['orient']} | {m['note']} |")
    lines += ["", f"**Total printed: {tot_m / 1000:.2f} kg PETG, ≈ {tot_h:.0f} h of printing** (+15% for supports, brims and failed prints → buy {math.ceil(tot_m * 1.15 / 1000)} kg + 1 kg of white or graphite for the finish you pick).", ""]
    lines += ["## Bought / non-printed parts shown in CAD", "", "| Part | Material |", "|---|---|"]
    for m in manifest:
        if not m["printed"]:
            lines.append(f"| `{m['name']}` | {m['material']} |")
    lines += ["", "## Arm mass properties used in the torque check (CAD shells + hardware)", "",
              "| Body | Printed g | Hardware g | Total g | CoM (mm, body frame) |", "|---|---|---|---|---|"]
    for b, mp in mass_props.items():
        lines.append(f"| {b} | {mp['printed_g']:.0f} | {mp['hardware_g']:.0f} | {mp['mass_g']:.0f} | {mp['com']} |")
    lines += ["", "Servos are not in these numbers; the sim adds them at their real positions.", ""]
    lines += FINISH
    (ROOT / "docs" / "PRINT_LIST.md").write_text("\n".join(lines) + "\n")


FINISH = """## Print settings (structure)

- PETG, 0.4 mm nozzle, 0.20 mm layers (0.12 mm for caps, coupler and gripper), 6 walls (2.4 mm = the shell wall), 40% gyroid infill where solid.
- Shell covers print **outer face down on the smooth PEI plate**. This gives the best cosmetic surface and needs no supports.
- Heat-set M3×4 inserts go in the cover bosses. M3 screws come from the inner face, so no screw heads are visible.
- Seams sit on the fillet tangent line and read as a design line. Puck seams are hidden under the grey end caps.

## Finishing (both colourways)

1. **Sand.** Wet-sand the visible faces 220 → 400 grit. Knock off layer lines and the seam ridge. Don't sand the joint faces or the bearing seats.
2. **Fill.** Apply a 2K or automotive spot putty to pinholes and the Z-seam, then sand 400.
3. **Prime.** 2–3 light coats of filler primer (high-build, grey). Sand 400 → 600 between coats until there are no visible layer lines.
4. **Paint.**
   - *Matte graphite*: satin/matte dark grey (≈ RAL 7021/7024) for all shells and caps. Caps can stay ~10% lighter for contrast.
   - *Matte white*: matte white (≈ RAL 9003) for shells; mid-grey (≈ RAL 7037) for end caps, the J1 collar, the lock sleeve and the roll flange.
5. **Clear.** 2 coats of matte 2K or acrylic clear (wear and UV resistance). Let it cure 48 h before assembly. Mask the LED diffuser, coupler contact faces, bearing bores and threaded inserts.
6. **Smoke cap.** Print in smoked/translucent grey PETG at 100% infill and 0.12 mm layers with a slow outer wall. Wet-sand 800 → 1500 and polish (or one coat of gloss clear) for the smoked-glass look.
7. **Fingers.** Laser-cut stainless gives the steel look for free. For PETG fingers, use primer + metallic silver + satin clear.
"""

if __name__ == "__main__":
    main()
