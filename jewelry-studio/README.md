# Jewelry studio around the Gantry Arm

A full jewelry-making bench (2.7 × 1.48 m) built around the Gantry Arm. **Design concept only. Nothing here is bought or approved.**

| File | What |
|---|---|
| `cad/studio.py` | Every station as CadQuery geometry → `cad/out/*.stl`, `manifest.json`, `layout.json` |
| `render/render_studio.py` | Cycles renders that reuse the Gantry Arm renderer and its detail materials: `--view overview / cold / hot` |
| `render/plan.py` | Labelled plan view → `render/out/studio_plan.png` |
| `render/out/` | `studio_overview.png`, `studio_cold.png`, `studio_hot.png`, `studio_plan.png` |

## The one rule: the robot does cold work, a person does hot work

A **steel heat-shield wall** at y = −250 mm splits the bench. The arm sits on the rail in front of it. Its links are printed PETG (softens ~80 °C) and its payload is 500 g, so it never touches the kiln, flasks, crucible, molten metal or the flame. The wall is outside its reach.

## Stations

### Cold-work row (robot-served)

| Station | What the robot does |
|---|---|
| **Bambu P1S** | Prints castable wax/resin patterns; the arm unloads the plate (see the candle-cell work) |
| **Servo vise** (0, 205) | Holds a part while the arm sands and polishes it with the compliant spindles |
| **Tool rack** | 3 spindles (240 / 600 grit, felt polish), automatic tool change |
| **Pickle pot** (citric acid) | Dips a cooled, quenched casting in a PP basket (never steel tongs in pickle) |
| **Ultrasonic cleaner** | Removes polishing compound between grits |
| **Vibratory tumbler** | Burnishes batches with ceramic or steel media; the arm loads and unloads it |
| **Inspection camera** | Macro camera + ring light + turntable: photographs each piece for QC and listings |
| **Precision scale** | Weighs metal before casting and finished pieces (for pricing) |
| **Bench polisher** | Part-to-wheel polishing: the arm holds the piece against a cloth or felt buff, with hoods catching debris |
| **Laser engraver** | Enclosed with an orange filter window; the arm loads through the side opening; engraves names and hallmarks |

### Hot zone (a person only, behind the wall)

| Station | Notes |
|---|---|
| **Vacuum investing** + wax tree on a rubber sprue base + steel flask | Investment is silica dust: mask and ventilation |
| **Burnout kiln** | Programmed ramp to ~730 °C; vent the fumes |
| **Electric melting furnace** + crucible tongs | Temperature controlled, no open flame to aim |
| **Vacuum casting machine** + pump | The person pours |
| **Fixed torch station** | See below |
| **ABC fire extinguisher** | At the end of the hot zone, within reach |

### Hand bench (a person)

Bench pin, ring mandrel, and a velvet tray of finished silver and gold rings.

## The fixed torch station (your torch-bracket idea)

The torch is **clamped in a steel bracket on its own steel stand**. It is not on the arm. The work moves; the flame doesn't. Around it:

- a **fire-brick turntable** for soldering and annealing;
- a **gas solenoid** that closes on power loss;
- a **piezo igniter** and **flame sensor** (no flame → gas off);
- a **fume hood** overhead;
- the **extinguisher** next to it.

This is the same architecture industrial torch cells use. Even with all of it, a person stays at the station while it's lit. The interlocks back the person up; they don't replace them.

## What is concept vs verified

- **Verified in sim/CAD earlier:** the arm, gantry, tool rack, spindles and vise pose (the IK solves the sanding pose used in the renders).
- **Concept only:** the cold-row station positions (y 380–600 and the polisher/laser at x 640/1010) are placed in the arm's **raised reach band**. Each needs an IK check and probably a riser. The laser load side at x ≈ 860 is at the edge of reach.
- **Appliances are drawn as generic shapes,** not specific products. They are not sized or priced, and none are in the ~$1,130 CAD budget.

## Hot-work safety list

ABC extinguisher within reach · leather apron and gloves · face shield / safety glasses · ventilation for investment dust and burnout fumes · non-flammable bench surface under the hot zone · never alone with the torch or the melt · keep wax and solvents away from heat · pickle with citric acid only, never steel in it.

## Next steps (not started)

IK reach check per station (and risers where needed), jeweler jaw inserts for holding pieces at the polisher, a force-controlled polishing skill, and a BOM with Canadian sources.
