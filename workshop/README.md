# Wall workshop around the Gantry Arm

A general-purpose workshop on an **IKEA KARLBY-size countertop (1860 × 635 × 38 mm) against a wall**, with a **classic jeweler's half-moon arc** cut into the front edge at your seat, with **pegboard** above it and the hot gear on a **separate steel rolling cart**. No laser engraver. It covers 3D printing, small-part finishing, repairs and jewelry. **Design concept only: nothing is bought or approved.**

| File | What |
|---|---|
| `cad/workshop.py` | All geometry → `cad/out/*.stl`, `manifest.json`, `layout.json`. Also runs the arm **IK reach check** for every robot station → `reach.json` |
| `render/render_workshop.py` | `--view overview / seat / robot / wall / hot` (reuses the Gantry Arm renderer and its detail materials) |
| `render/plan.py` | Top view + wall elevation → `render/out/workshop_plan.png` |

## Layout (you stand at the front edge)

```
WALL ─ steel heat panel ─┬──────────────── pegboard (1.8 x 0.9 m) ─────────────────┐
  [cart: kiln | torch]   │ ctrl   ════ X rail + Z gantry ════            [ P1S   ] │
  extinguisher (far end) │ mat  ultrasonic tumbler cam scale VISE pickle  rack     │
                         │  (  half-moon arc + pin = your seat  )                    │
```

- **Right end:** the Bambu P1S, turned so its door faces the arm. The arm pulls the build plate by its tab.
- **Middle:** the robot stations, all inside the arm's reach.
- **Left end, your seat:** the half-moon arc (see below), with a small bench vise, cutting mat and steel bench block behind it. This end is outside the arm's reach on purpose, so the arm never works where your hands are.
- **Legs:** two round steel legs hold the left end, and a 6-drawer unit (ALEX-style) carries the right end under the P1S.

## The half-moon jeweler's seat

| Part | Detail |
|---|---|
| Arc | 300 mm wide × 150 mm deep, jigsawed into the KARLBY front edge, centred 230 mm from the left end |
| Bench pin | Classic V-slot pin in a steel holder under the top, at the back of the arc, sticking out toward you |
| Sweeps tray | Steel tray on runners under the arc. It catches filings and dropped stones, and slides out to empty |
| Leather skin | Inside the tray, for a soft landing |

Tips for the cut:
- Mark the arc with a string compass.
- Cut with a fine-tooth jigsaw blade from the underside (less veneer chipping), then sand.
- Seal the cut edge with oil or varnish: the core is exposed there and must not take up water.
- A jeweler's bench pin usually sits at about armpit height when seated, higher than a desk. Adjustable legs, or a raised seat pad on your chair, get you closer to that.

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

The heat-shield wall and the laser engraver are gone. The bench is now an IKEA top with a half-moon arc. The bench polisher is gone too: the robot polishes with the felt spindle in the vise instead. The kiln and torch moved onto the cart. The pegboard replaced the back half of the old bench.

## Not done yet

- Collision-free path planning between stations.
- Hook positions matched to the pegboard brand you buy.
- A BOM with Canadian sources.
