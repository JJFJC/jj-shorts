# Learn CAD with Autodesk Fusion, using this robot

The path uses real parts of the Gantry Arm as exercises. Every exercise has a matching STEP file in `cad/out/step/`, so you can import it and check your model against it. The numbers come from `params.py`.

**Setup:** install Fusion with the free **Personal Use** licence (non-commercial; it limits the number of editable documents and some advanced features). Then set the units to **mm**: *Preferences → Default Units → Design → mm*.

| # | Lesson | You learn | Exercise (from this robot) | Check |
|---|---|---|---|---|
| 1 | Find your way around | orbit, pan, zoom, timeline, browser, **Inspect → Measure** | Open `cad/out/step/gripper_housing.step` and measure its diameter (58), height (38) and wall (2.4) | Your numbers match |
| 2 | Sketch → extrude | sketches, **constraints** (fully black lines = fully defined), dimensions, extrude / cut | Model the **fit coupon** (96 × 74 × 6 plate, 5 rows × 3 holes) | Compare with `cad/out/test_prints/fit_coupon.step` |
| 3 | Parameters | *Modify → Change Parameters*, naming dimensions | Make the coupon's hole sizes parameters (`insert_d`, `magnet_d`…). Change one and watch the model update. This is the same idea as `params.py` | Change `insert_d` to 4.1 and only the insert holes change |
| 4 | Revolve, shell, fillet | revolve, shell (2.4 mm wall), fillet | `j4_cap_a` (disk Ø50 × 2) and the `gripper_housing` cylinder | Measure against the STEP |
| 5 | Tangent arcs + split body | tangent lines between circles, symmetric extrude, **Split Body** on a plane | The `forearm` link: circles R38 / R32, 150 apart, joined by tangent lines, 26 thick, R7 edges, shelled to 2.4 mm. Then split it into body and cover | Overlay `forearm_body.step` + `forearm_cover.step` |
| 6 | Components and joints | components vs bodies, **Joint** (revolute), motion | Import `coupler_robot_half` + `coupler_tool_half` + `coupler_lock_sleeve`. Joint the sleeve so it rotates like the bayonet lock | The sleeve turns about the tool axis |
| 7 | Design for printing | heat-set insert holes (use your coupon result), 0.4 mm chamfer on the bed edge (elephant's foot), orientation, export **3MF** → Bambu Studio | Re-make `tool_pen_holder` with your insert size, then slice and print it | It prints, the pen fits, the insert holds |
| 8 | Capstone: a real contribution | everything above | Design a **new quick-change tool**: start from `coupler_tool_half.step` (keep the cone, lugs, 4 pad holes and the parking groove), then add your own tool body. Ideas: magnifier, marker cap holder, small scoop, phone stand | It fits the tool rack slot (groove at z 10–14 mm, Ø56) and weighs < 120 g (J4 is the limit) |

**After that**, the next real job on this robot is in your reach: **design the servo pocket inside the gripper housing** once an STS3215 is in your hands. Measure the servo with calipers, model a cradle, and test-print just that piece.

## Habits that save hours

- **Fully constrain every sketch.** Blue lines mean "can still move". Unconstrained sketches break when you change a number later.
- **Name your parameters and dimensions.** Future-you won't remember what `d17` was.
- **Model in the print orientation, or know it.** Flat face down, no overhangs above 45° without a chamfer, holes vertical where possible.
- **Print a test piece of only the tricky feature** before the whole part (that's what the coupon is for).
- **Save versions.** Fusion keeps a history: use *Save* with a short comment at each milestone.

## Good free resources

- Autodesk's own Fusion learning pages and tutorials (in the Fusion app: *Help → Learning and Documentation*).
- YouTube: **Lars Christensen** (Fusion basics, very beginner-friendly) and **Product Design Online** (project-based courses).
- When a lesson clicks, send me a screenshot or your exported STEP. I can check it against the robot's dimensions and tell you what's off.
