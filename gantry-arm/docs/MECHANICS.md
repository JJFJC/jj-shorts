# Gantry Arm — inside mechanics

Joint-by-joint design of what is inside the shells: servos, gears, bearings, shafts, couplings, mounts and cable path.
CAD: `cad/internals.py` (62 parts, each in its own body frame) and `cad/servo_pack.py` (servo packing). Numbers come from `params.py`. Renders are `render/out/mech_*.png`, made by `render/render_mechanics.py`.

**Status:** design and simulation only. All internal parts and shells pass a clash check with zero overlaps (stator frames, arm straight). Items marked **VERIFY** need a real part in hand.

## Summary

| Joint | Drive | Bearings | Load at full reach, 500 g (static) | Notes |
|---|---|---|---|---|
| J1 base yaw | 1× STS3215, inverted in the turret, **Oldham coupling** to a fixed spigot | 2× 61812-2RS (60/78×10) on the spigot, 23 mm apart | 4.6 N·m bending moment, carried by the bearings | The servo sees torque only |
| J2 shoulder | 2× STS3250 on **steel m1 pinions 18T + 17T → 36T hub gear** | 2× 6804-2RS on a Ø20/15 hollow shaft | 13% of stall, 41% of rated | Spring still fitted |
| J3 elbow | 2× STS3250 on **m1 15T + 14T → 30T hub gear** | 2× 6803-2RS on a Ø17/13 hollow shaft | 13% of stall, 39% of rated | Was 75% of rated with direct drive |
| J4 wrist pitch | 1× STS3215, on the axis, body pointing at the tool | 6806-2RS around the horn | 31% of stall, 92% of rated | Unchanged; the weakest joint now |
| J5 wrist roll | 1× **STS3032 micro** (6 V), off the axis, **1:1 m0.5 36T pair** | 6805-2RS on the roll shaft | 17% of stall, 50% of rated | A buck converter in the wrist makes the 6 V |

## Why the shoulder and elbow are geared pairs

The links are one-sided: the upper arm is on +Y and the forearm on −Y. With two servos "facing outward", only one of them could reach the link, so the second servo drove nothing. Owner's choice (2026-11-20): a **geared pair**. Both servos sit in the puck, each turns its own pinion, and both pinions mesh one hub gear fixed to the link.

- **Torque:** 2 servos × 2:1 × 95% mesh efficiency is about 3.8× one servo. J2 drops from 77% to 41% of rated, J3 from 75% to 39%.
- **Stiffness:** a position servo seen through an n:1 gear is n² as stiff at the joint, so the pair is 8× one direct servo. TCP droop at full reach with the default P gain drops from 37 mm to 15 mm, and is about 0.6 mm with gravity feed-forward (`docs/SIM_REPORT.md`).
- **Resolution:** 4096 steps/rev at the servo gives 0.044° at the joint.
- **Speed:** the joint turns at half the servo speed. My earlier "~225°/s" was an unverified estimate. Check the STS3250 datasheet speed at 12 V and halve it.
- **Backlash:** taken out by **preload**. The two servos are commanded ±0.4° (motor side) apart, so they push against each other through the hub gear. Each pinion then always rests on one tooth flank. Measure the real value on the assembled joint: jog the pair apart until Present_Load rises on both, then add ~0.15°. `config.geared_preload_deg` holds it.

### Gear strength (Lewis, static ×1.5 for dynamics, worse servo carrying 59%)

| Mesh | Hub radius | Tooth force | Face | Bending stress | PETG allowable | Steel |
|---|---|---|---|---|---|---|
| J2 18T/17T → 36T | 18 mm | ~118 N | 10 mm | ~38 MPa | ~10 MPa ✗ | ✓ |
| J3 15T/14T → 30T | 15 mm | ~138 N | 10 mm | ~55 MPa | ✗ | ✓ |

That is why the gears are **bought steel spur gears** (module 1, 20° pressure angle), not printed. The 14T and 15T pinions are below the 17-tooth undercut limit, so buy profile-shifted (+0.2) pinions or accept slightly weaker roots. The stress above already uses a reduced form factor.

### Absolute joint angle at power-up: vernier pinions

The servos turn more than one revolution over the joint range (J2 235° → 470°, J3 280° → 560°). An STS servo only knows its angle within one turn, so after power-up the joint angle is ambiguous.

The owner asked for joint encoders. An on-axis AS5600 needs a magnet **on the joint axis**, and that is exactly where the hollow wiring shaft runs. Off-axis magnetic sensing exists, but it needs special ring magnets and tight tolerances ([MPS side-shaft notes](https://www.monolithicpower.com/en/learning/resources/using-magalpha-rotary-magnetic-sensors-in-side-shaft-mode), [SimpleFOC forum](https://community.simplefoc.com/t/encoder-selection-for-mini-robotic-arm/6735)).

So each pair uses **two different pinions** (18T + 17T on J2, 15T + 14T on J3). The servos turn at 2.000× and 2.118× (J2), or 2.000× and 2.143× (J3), the joint angle. Only one joint angle in the range fits both single-turn readings, like the vernier scale on a caliper.

- **Separation:** the wrong candidates sit 21° (J2) or 26° (J3) away at servo B.
- **Thresholds:** the decoder accepts a fit within 4° and requires the next candidate to be 6° worse.
- **Tested:** the decoder recovers the angle over the whole range with ±0.6° noise (`tests/test_dynamics.py`).
- **Cost:** no extra hardware.
- **Slip detection:** the same two readings detect a loose pinion or a skipped tooth while running (`GearedPair.check`).

What this does **not** give: an angle measured after the gears, independent of them. With fixed-centre steel gears and preload that is acceptable. A gear failure shows up as the two servos disagreeing.

**VERIFY:**
- **Multi-turn mode:** the plugin writes position limits 0/0 to the paired servos for Feetech multi-turn mode. Test this on one servo on the bench before mounting.
- **Bus driver:** check that LeRobot's bus driver accepts the STS3032 model name (J5).

## J1 base yaw

The old CAD put the J1 servo on the axis with a 7 mm wiring bore. But an STS case needs a **36 mm radius** around its output shaft, because the shaft sits 11.5 mm from one end. The 4.6 N·m bending moment from the arm also has to go into bearings, not the servo spline.

| Part | Frame | What it does |
|---|---|---|
| `j1_spigot` (printed PETG) | fixed (z_carriage) | Ø60 tube screwed to the base floor. Carries the inner races of both 61812 bearings. Its top plate has an off-centre cable hole. |
| 2× 61812-2RS (Ø60/78×10) | fixed | 23 mm apart. Moment 4.6 N·m → about 200 N per bearing, tiny for this size. |
| `j1_turret_skirt` + collar bore (printed) | turret | Outer races: lower in the skirt, upper in the collar bore (Ø78) |
| Oldham coupling (lower hub fixed, POM slider, upper hub on the horn) | both | Passes torque only. Takes up any misalignment, so the servo never sees the moment. |
| STS3215, inverted, in `j1_servo_cradle` | turret | Rides with the turret. Its case top is at z 93, below the shoulder servos. |

Wiring: through the base floor, up inside the spigot, out through the top-plate hole, then a ±150° service loop around the Oldham stack, past the cradle into the shoulder puck (orange harness in the renders).

## J2 shoulder and J3 elbow (same layout)

Stack along the joint axis (Y), from the link side (+Y) inward:

| y (mm) | Part |
|---|---|
| 46–48 | outer end cap (ring around the shaft) |
| 31–45 | bearing boss (printed, glued in the puck) with 2 bearings back to back |
| 20–30 | **gear layer**: hub gear on the hollow shaft + 2 pinions |
| 16–20 | aluminium servo horns; the pinions bolt to them (4× M2.5 on the horn pattern, **VERIFY**) |
| 11–16 | spline bosses + the **cable service loop** coiled around the shaft end |
| −24–11 | the two servo cases, packed by `cad/servo_pack.py` |
| −27–−24 | printed mount plate (M2 into each case's back holes) |

- **Hollow shaft:** aluminium tube, J2 Ø20/15 and J3 Ø17/13. It is clamped in the link hub with 2× M3 set screws on a flat. The harness runs inside it from the puck into the link, so the joint axis carries the wires. That is why the angle comes from the vernier, not an AS5600.
- **Servo packing:** each output shaft must sit on its pinion's centre (27.0 / 26.5 mm from the axis at J2, 22.5 / 22.0 at J3). The packer maximises the smallest gap. The result is ≥ 1.1 mm between cases and ≥ 2.7 mm to the puck wall. At J2 the cases also stay above the J1 servo.
  - **VERIFY:** the packing models the bare case only. The servo connector and cable exit add a few mm at one end. Check the real servo before printing the mount plate.
- **Elbow puck radius:** grew from 40 to 44 mm (`STYLE["elbow_puck_r"]`) so the 2:1 pair fits.

## Wrist (J4 pitch, J5 roll)

The old roll housing (radius 28) could not hold an STS3215 for J5. The 36 mm envelope rule applies here too, and J4's body takes the middle of the wrist.

- **J4:** STS3215 on the pitch axis, horn toward the forearm (−Y). Its body points at the tool (+z), the only direction with room.
  - The **6806 bearing** sits in the −Y wall **around** the horn, not in series with it, so the J4 stack stays short.
  - The bearing's inner race is on a turned aluminium hub ring, part of the forearm. The horn bolts to that ring's flange.
- **J5:** owner's choice (2026-11-20). A Feetech **STS3032** micro bus servo (0.44 N·m stall at 6 V, needs 0.07) sits **beside** J4.
  - It drives the roll shaft through a **1:1 pair of m0.5 36T gears** on 18 mm centres.
  - A **6805** bearing at the housing tip carries the tool moment.
  - A small **12 V → 6 V buck** powers only this servo. The data line is shared with the bus.
  - The roll housing radius grew from 28 to 30 mm.
  - **VERIFY:** vendors list two sizes for the STS3032 (23.2×12.1×28.5 and 32×12×27.5 mm). The CAD uses the first. The second would not fit beside J4.

## Open items (found in this pass, not solved yet)

1. **Shoulder spring routing.**
   - **Problem:** the zero-length spring needs a cable from a pulley on the turret to a point on the upper arm. There is only a 2 mm gap between the puck face and the link.
   - **Options:** a cam inside the link hub, or a spring pod on the −Y end cap (that needs the shaft through the full puck width).
   - The sim assumes the ideal spring.
2. **J4 wrist harness.** The J4 servo sits on the pitch axis, so the cable can't pass through it. It needs an external service loop between the forearm and the wrist (±110°) under a small flexible sleeve. Not modelled yet.
3. **J4 is now the weakest joint:** 92% of rated torque at full reach with 500 g. Keep holds short, or use lighter tools at full reach.
4. **Real-part checks** (all marked VERIFY above): STS3032 size, multi-turn mode, servo connector clearance, horn bolt pattern, LeRobot support for the STS3032.

## Renders

| File | What |
|---|---|
| `mech_overview.png` | Whole arm with its front half cut away just past the gear layer: both twin-pinion gear trains face-on |
| `mech_gears.png` | Close-up of the J2 and J3 gear trains |
| `mech_shoulder.png` | J1 + J2 cut on the arm plane: spigot, bearings, Oldham, inverted J1 servo, twin pinions, hub gear, hollow shaft, harness |
| `mech_wrist.png` | Wrist cut on its mid-plane: J4 servo, 6806 bearing, J5 micro servo, 1:1 roll gears, roll bearing |
| `mech_exploded.png` | Exploded shoulder in assembly order |
