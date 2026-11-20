# Gantry Arm — industrial design notes

This is an original design in an industrial-cobot style. It copies no brand's shapes, logos or proportions. Renders are in `render/out/`.

## Language

| Element | Rule | Where in CAD (`cad/parts.py`) |
|---|---|---|
| Joint pucks | Round housings whose axis is the joint axis. Recessed end caps in a contrasting tone. | `j2_shoulder_puck`, `j3_elbow_puck`, wrist shell + `j*_cap_*` |
| Links | Hollow, tapered capsule side-plates (26 mm thick, R7 edges), offset **beside** the pucks: the upper arm on +Y, the forearm on −Y. | `stadium()`, `upper_arm_*`, `forearm_*` |
| Base | J1 drum + grey collar, with the shoulder puck sitting across the top. | `j1_base_housing`, `j1_turret_collar` |
| Accent | 5 mm LED channel on the outer face of the upper arm (WS2812, 6 LEDs) under a translucent diffuser. | `upper_arm_led_diffuser` |
| Tool end | Grey roll flange → magnetic coupler + bayonet lock sleeve → tool. | `j5_roll_flange`, `coupler_*` |
| Gripper | Compact Ø58 cylinder, smoked translucent cap, two stainless V-linkage fingers with TPU pads. | `gripper_*` |
| Gantry | Black anodised extrusions with flush cover strips; covered X and Z carriages; the cable chain is hidden in a rear trough (X) and a rear column cover (Z); end housings; controller box with a hardware e-stop. | `build_gantry()` |

## Eyes head

A stereo USB camera (60 mm baseline) sits in a capsule head on the Z-column top, with a 12-LED WS2812 ring around each lens. It rides on the X carriage, so the vision calibration follows `gantry_x`. It gives depth for picking and replaces the plain overhead camera. The rings show the same status colours as the arm strip, and can blink or "look". Chosen by the owner over the ring-only and no-head options.

## Tool rack (passive tool changer)

Two printed 2-slot modules (`tool_rack_1/2`) screw to the table inside the work area.
- **Holding the tool:** each tool's coupler half has a parking groove. The rack fork slides into it horizontally.
- **Unlocking the coupler:** a cam channel on the rack post holds the bayonet sleeve's lever open while the tool is docked.
- **No actuators on the rack:** the Z gantry provides the force to break the magnets.

The sequence is in `skills/tool_changer.py`, and `docs/img/demo_tool_change.png` shows it on the sim.

## LED status colours (firmware/plugin, Step 3)

| State | Colour |
|---|---|
| Idle / torque on | steady cyan-blue |
| Moving | slow green pulse |
| Homing | amber chase |
| Fault / e-stop | fast red blink |

## Enclosed servos and internal wiring

Every servo sits inside a puck or housing; no servo body is visible. The full inside design is in **`docs/MECHANICS.md`** (geared twin-servo pairs on J2/J3, J1 bearing spigot with an Oldham coupling, the wrist stack with the STS3032 roll servo).

- **J2/J3:** the harness runs through the **hollow joint shafts** (Ø20/15 and Ø17/13) from the puck into the link, with a coiled service loop inside each puck.
- **J1:** the harness passes off-axis through the fixed spigot, with a ±150° loop around the Oldham coupling.
- **J4:** the servo sits on the pitch axis, so the wrist harness needs an external loop (open item).
- No slip rings: every joint has limited travel (`params.JOINT_LIMITS`), and J5 roll is limited to ±180°.
- 12 V and the bus continue through the coupler's pogo pins: 12 V, GND, DATA, TOOL_ID, plus 2 high-current contacts.

## Shell splitting and fasteners

- Links split on the fillet tangent line of the outer face. The cover is a lid with the cosmetic face printed down on smooth PEI, so the seam reads as a design line.
- M3 screws go in from the inner face, **only in the hub zones hidden by the neighbouring puck**, into M3×4 heat-set inserts in bosses on the cover. The visible middle of each link is held by two snap clips. An earlier version had screws mid-span, and they showed in the renders.
- Puck seams are hidden under the end caps. The wrist shell splits on its mid-plane.
- Every printed part fits the P2S bed. The upper arm halves (251 mm) print rotated on the bed diagonal. Long gantry covers are segmented (4 trough segments, 3 column-cover segments) with lap joints.

## Mass impact

The shells weigh more than the Step 1 lumped estimates. `cad/build_all.py` writes measured mass properties, and `sim/static_check.py` uses them, so `docs/SIM_REPORT.md` always reflects the real shells.
