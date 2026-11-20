# Wall workshop around the Gantry Arm

A general-purpose workshop on a **1.8 × 0.6 m bench against a wall**, with **pegboard** above it and the hot gear on a **separate steel rolling cart**. No laser engraver. It covers 3D printing, small-part finishing, repairs and jewelry. **Design concept only: nothing is bought or approved.**

| File | What |
|---|---|
| `cad/workshop.py` | All geometry → `cad/out/*.stl`, `manifest.json`, `layout.json`. Also runs the arm **IK reach check** for every robot station → `reach.json` |
| `render/render_workshop.py` | `--view overview / robot / wall / hot` (reuses the Gantry Arm renderer and its detail materials) |
| `render/plan.py` | Top view + wall elevation → `render/out/workshop_plan.png` |

## Layout (you stand at the front edge)

```
WALL ─ steel heat panel ─┬──────────────── pegboard (1.8 x 0.9 m) ─────────────────┐
  [cart: kiln | torch]   │ ctrl   ════ X rail + Z gantry ════            [ P1S   ] │
  extinguisher (far end) │ mat  ultrasonic tumbler cam scale VISE pickle  rack     │
                         │ bench vise / bench pin (your end)                       │
```

- **Right end:** the Bambu P1S, turned so its door faces the arm. The arm pulls the build plate by its tab.
- **Middle:** the robot stations, all inside the arm's reach.
- **Left end:** your hand-work area (cutting mat, small bench vise, bench pin). This end is outside the arm's reach on purpose, so the arm never works where your hands are.
- **Under the bench:** a shelf with two storage bins.

## What the robot does (IK reach checked for each station)

| Station | Jobs | Reach |
|---|---|---|
| Servo vise | Sand, polish, deburr or drill any small part: prints, wood, metal, jewelry | ✅ |
| Tool rack | Picks up and swaps its 3 spindles (240 grit, 600 grit, felt polish) | ✅ |
| P1S | Unloads prints | ✅ |
| Ultrasonic cleaner | Parts, jewelry, printer nozzles | ✅ |
| Vibratory tumbler | Deburrs and polishes batches | ✅ |
| Camera + turntable | QC photos, measuring, photos for listings | ✅ |
| Scale | Weighs parts, counts screws, checks how much filament is left | ✅ |
| Pickle crock (citric) | Cleans metal after soldering | ✅ |

The reach check covers position only (gripper pointing down). Collision-free paths between stations still need to be planned in the sim.

**Measured reach on this bench:** x ±650 mm along the rail and y up to about 400 mm at station height. There is no reach at x ±800, so the bench ends are for you and the printer.

## Pegboard wall (for you)

| Zone | Contents |
|---|---|
| Hammer + screwdrivers | Claw hammer, 3 drivers |
| Pliers | 3 pairs |
| Saws + files | Hacksaw, jeweler's saw, 4 files |
| Measuring | Steel rule, calipers, square, tape |
| Parts bins | 6 bins on a rail: screws, magnets, heat-set inserts |
| Shelf | Glue, oil, polishing compound; filament spool on a rod |
| Holding + power | 2 F-clamps, cordless drill in a holster |
| Spools | Solder and wire on a rod |
| PPE | Safety glasses, ear muffs |
| Paper towel | |
| Lighting | LED bar along the top |

The tools stick out at most about 110 mm from the board, keeping them clear of the gantry.

## Hot cart (you only)

- **What's on it:**
  - 600 × 450 mm steel cart with a cement-board top.
  - **Top:** a small kiln and the **fixed torch station**. The torch is clamped in a steel bracket over a fire-brick turntable, with a gas shut-off valve, a flame sensor and a fume hood ducted into the wall.
  - **Lower shelf:** melting furnace, small vacuum caster, flasks, gas canister.
- **Behind it:** a **steel heat panel**, not pegboard. Hardboard pegboard is flammable.
- **Fire extinguisher:** on the wall at the **far side of the cart**, away from the flame, so you never reach past the torch for it.
- **Rolls away:** the cart moves when you need floor space, and outside when you cast.

## Changes from the jewelry studio

The heat-shield wall and the laser engraver are gone. The bench polisher is gone too: the robot polishes with the felt spindle in the vise instead. The kiln and torch moved onto the cart. The pegboard replaced the back half of the old bench.

## Not done yet

- Collision-free path planning between stations.
- Hook positions matched to the pegboard brand you buy.
- A BOM with Canadian sources.
