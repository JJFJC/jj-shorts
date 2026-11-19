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

## LED status colours (firmware/plugin, Step 3)

| State | Colour |
|---|---|
| Idle / torque on | steady cyan-blue |
| Moving | slow green pulse |
| Homing | amber chase |
| Fault / e-stop | fast red blink |

## Enclosed servos and internal wiring

- Every servo sits inside a puck or housing. No servo body is visible.
- All wiring goes through the hollow links and Ø14 mm bores at each joint centre. The links have a 2.4 mm wall with ~20 mm of clear inside, room for the 3-wire TTL bus, 12 V and the LED data line.
- STS servos are not hollow-shaft, so the cable passes beside the horn on the idler side of each puck, with a ±150° service loop coiled inside the puck. No slip ring is needed, because every joint has limited travel (`params.JOINT_LIMITS`). J5 roll is limited to ±180° for this reason.
- 12 V and the bus continue through the coupler's 4 pogo pins: 12 V, GND, DATA, TOOL_ID.

## Shell splitting and fasteners

- Links split on the fillet tangent line of the outer face. The cover is a lid with the cosmetic face printed down on smooth PEI, so the seam reads as a design line.
- M3 screws go in from the inner face (hidden against the pucks) into M3×4 heat-set inserts in bosses on the cover.
- Puck seams are hidden under the end caps. The wrist shell splits on its mid-plane.
- Every printed part fits the P2S bed. The upper arm halves (251 mm) print rotated on the bed diagonal. Long gantry covers are segmented (4 trough segments, 3 column-cover segments) with lap joints.

## Mass impact

The shells weigh more than the Step 1 lumped estimates. `cad/build_all.py` writes measured mass properties, and `sim/static_check.py` uses them, so `docs/SIM_REPORT.md` always reflects the real shells.
