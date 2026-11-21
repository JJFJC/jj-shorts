# Adding a kiln (lost-wax casting) and the question of the blow torch

## Where a kiln fits: lost-wax casting for jewelry

| # | Step | Who / what | Notes |
|---|---|---|---|
| 1 | Make patterns | **P1S + robot** | Use castable wax filament or castable resin; regular PLA burns out dirty and leaves ash. Robot unloads and does light cleanup with the spindle tools |
| 2 | Sprue the patterns into a tree on a rubber base | You (or the robot with the gripper) | Delicate but light work |
| 3 | Mix and pour investment into a steel flask, vacuum to remove bubbles | **You** | Wet, time-critical; wear a dust mask (silica) |
| 4 | **Burnout** | **Kiln, programmed ramp** (~730 °C peak, several hours; follow the investment maker's schedule) | Runs on its own; vent the fumes |
| 5 | Melt + cast (vacuum or centrifugal) | **You** | Electric melting furnace recommended over a torch (see below) |
| 6 | Quench, break out, cut sprues | You | |
| 7 | Sand + polish the metal | **Robot** spindle tools with metal-grade bits and compounds, or a tumbler with steel shot | Same tool rack, different bits |

**The robot does not handle the kiln or the flasks.** Flasks leave the kiln at 400–700 °C and weigh 0.5–1.5 kg. The arm is printed plastic (PETG softens at ~80 °C), and its payload is 500 g.

## Can the robot fully use a blow torch? No; don't do it

- **Open flame on a plastic robot near wax and a gas bottle is a real fire and burn hazard.** Industrial torch robots work inside metal enclosures with flame sensors, gas solenoid shut-offs, interlocks and fire suppression, and even there, **pouring molten metal is a person's job or a dedicated machine's.**
- **Safer and more repeatable:** an **electric melting furnace** (digital crucible, ~$300–600 CAD) plus a **vacuum casting machine.** Electric heat is temperature-controlled, with no open flame to aim.
- **If you torch by hand:** the robot can hold a part at a *fixed* station at a safe distance (metal holder, not printed), or do the cold work afterwards.

## Workshop safety for hot work

ABC fire extinguisher within reach · leather apron and gloves · face shield / safety glasses · good ventilation (investment dust, burnout fumes) · non-flammable work surface · never alone · keep wax and solvents away from heat sources.
