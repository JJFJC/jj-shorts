# Gantry Arm — assembly guide (inside mechanics)

The order to build the joints described in `docs/MECHANICS.md`. Part names match `cad/internals.py` and `cad/parts.py`.
**Nothing here is bought yet.** This is the plan for when the parts arrive.

**Tools:**
- Hex keys 1.5–2.5 mm and a soldering iron with heat-set insert tips.
- Thread-locker (medium), retaining compound (bearing fit), grease (gears), cable ties, heat-shrink.
- Feetech USB adapter and a PC with the plugin installed (`software/`).

## 0. Before assembly: bench-test the servos

| Step | Why |
|---|---|
| Give every servo its bus ID with `lerobot-setup-motors`: J1 = 1, J2 = 2 + 3, J3 = 4 + 5, J4 = 6, J5 = 7, gripper = 8 | Easier now than inside the puck |
| On one STS3250: set Min/Max position limit to 0, command +500° and −500°, check it gets there and reports it | **VERIFY:** the multi-turn mode the geared pairs rely on |
| Power the STS3032 from the 6 V buck, check it answers on the shared bus next to a 12 V servo | **VERIFY:** the mixed-voltage bus |
| Measure each servo's case, connector position and horn bolt pattern against `params.STS3215.body` / `SERVO_SHAFT_FROM_END` | **VERIFY:** the packing has ~1 mm margins |

## 1. J1 base yaw

1. Press the two 61812 bearings onto `j1_spigot`: lower one on the shoulder at z 6, upper one at z 29. Use retaining compound and avoid loading through the balls.
2. Screw the spigot to the base floor (4× M3 into heat-set inserts). Feed the J1 → J2 harness up through the floor hole, the spigot and the top-plate hole.
3. Screw the lower Oldham hub to the spigot top plate.
4. Slide `j1_turret_skirt` + collar over both bearings. The skirt takes the lower outer race, the collar bore the upper one.
5. Mount the J1 STS3215 **upside down** in `j1_servo_cradle` (M2 into the case side holes). Bolt the upper Oldham hub to its horn.
6. Lower the turret onto the spigot with the POM slider between the two Oldham hubs. Turn it by hand: it must turn freely, without the servo binding.
7. Coil the harness once around the Oldham stack (±150° service loop) and tie it to the cradle rib.

## 2. J2 shoulder (repeat for J3 elbow with its parts)

1. Bolt the **18T** pinion to servo A's horn and the **17T** pinion to servo B's horn (4× M2.5, thread-locker).
   - At J3 these are the 15T (A) and 14T (B).
   - **Keep track of which is which:** the software expects servo ID 2 (and 4 at J3) on the 18T/15T pinion.
2. Screw both servos to `j2_mount_plate` in the positions from `cad/out/servo_layout.json`. The plate has the holes. Pass the bus cables through the plate's cable hole.
3. Press the two 6804 bearings into `j2_bearing_boss` back to back. Glue the boss into the puck's +Y wall.
4. Slide the 36T hub gear onto the hollow shaft and lock it with the set screw on the flat. Push the shaft through the bearings from inside the puck.
5. Lower the servo plate into the puck so both pinions mesh the hub gear. **Do not force a mesh.** Turn the hub gear a tooth until both pinions drop in.
6. Coil the harness once around the shaft end in the slab between the horns and the cases, then feed it into the hollow shaft.
7. Grease the gears lightly (a PTFE or lithium grease that is safe on plastics). Close the inner end cap.
8. Fit the upper-arm hub over the shaft end and tighten its 2× M3 set screws onto the flat. Close the outer end cap.

## 3. Wrist

1. Bolt the aluminium hub ring to the forearm's distal hub (it is part of the forearm). Press the 6806 bearing onto the ring.
2. Mount the J4 STS3215 in the wrist with its horn toward the −Y wall and its body toward the tool. Bolt the horn to the hub ring's inner flange.
3. Mount the STS3032 beside it (y ≈ +16) and fit the two 36T m0.5 gears: one on the servo, one on the roll shaft. Check the mesh by turning the roll shaft.
4. Press the 6805 bearing into the housing tip seat and fit the roll shaft. Bolt the roll flange to the shaft.
5. Fix the 6 V buck board to the housing wall. Wire 12 V in, and run 6 V out to the STS3032 only. The data line goes from the bus to the STS3032.
6. Route the wrist harness as an **external loop** between the forearm and the wrist under a flexible sleeve (open item, see MECHANICS.md).

## 4. Calibration on the robot

| Step | Command / action | Result |
|---|---|---|
| 1 | Arm **straight up**, gripper half open (the calibration pose) | — |
| 2 | `lerobot-calibrate --robot.type=gantry_arm` | Zero for every servo; the secondary of each pair gets the mirrored drive mode |
| 3 | Power-cycle, connect, read the log line `absolute joint angle ... from the vernier pair` for J2 and J3 | Must say ≈ 0° in the calibration pose. Repeat at the joint limits. |
| 4 | Preload: jog servo A of J2 away from B in 0.1° steps until Present_Load rises on both; put that + 0.15° in `geared_preload_deg` | Backlash-free J2/J3 |
| 5 | Run `software/demos/sim_demos.py` motions slowly (feed 20%) on the real arm, watching for the ganged fault | No "geared servos disagree" fault |

**If the vernier refuses** (`readings disagree` or `ambiguous`), a pinion was mounted on the wrong servo, or a tooth skipped during assembly. Check the 18T/17T (15T/14T) assignment and re-mesh.
