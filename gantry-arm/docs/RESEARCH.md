# Gantry Arm — Step 0 research

Checked 2026-11-19. Rule followed: look for existing open-source work before designing anything new.
Nothing here was copied wholesale. Where a file or number is reused, its source and license are named below.

## Summary

| Source | License | What we reuse | What we do NOT reuse |
|---|---|---|---|
| [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | Apache-2.0 | STS3215 horn/body dimensions for servo pockets; MuJoCo `sts3215` actuator class (`kp 17.8`, damping 0.60, armature 0.028, frictionloss 0.052) as the starting sim tuning; SO-101 leader arm as an optional teleop device | Link lengths (SO-101 is ~0.25 m shoulder-to-wrist, ours is ~0.45 m reach); the STL files themselves |
| [huggingface/lerobot](https://github.com/huggingface/lerobot) (v0.7.1, main @ 8ec1a47) | Apache-2.0 | `Robot` base class, `FeetechMotorsBus` (already supports `sts3250`, see `hope_jr_arm.py`), `ensure_safe_goal_position`, camera configs, calibration flow, plugin auto-discovery (`register_third_party_plugins`, prefix `lerobot_robot_`) | — (dependency, not vendored) |
| [lerobot_robot_lekiwi_pincopen](https://pypi.org/project/lerobot-robot-lekiwi-pincopen/) 0.1.2 | Apache-2.0 | Template for a mixed STS3215/STS3250 bus: replace a `Motor`'s model with `sts3250` per joint from a config list; write P-gain/accel tuning on every connect from config fields; opt-in per-tick motion cap (`max_relative_target`); MJPG camera fourcc | LeKiwi mobile base, PincOpen gripper, host/client network split |
| [PCrnjak/PAROL6-Desktop-robot-arm](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm) | CERN-OHL-S-2.0 (hardware, strongly reciprocal) | **Reference only**: joint layout and ranges, wrist arrangement. | No CAD/STL. Copying geometry would force our hardware under CERN-OHL-S. |
| [Source-Robotics/robot_descriptions](https://github.com/Source-Robotics/robot_descriptions) (PAROL6 URDF/MuJoCo) | CC-BY-4.0 models, Apache-2.0 tools | MJCF structure as a pattern (default classes per actuator, `flange` site at the tool) | Meshes (not needed; we generate our own from CadQuery) |
| [BerkeleyAutomation/RobotToolChanger](https://github.com/BerkeleyAutomation/RobotToolChanger) (`tool-changer` branch) | **No license file** (README license section is commented out) → all rights reserved | Concept only: cone-aligned coupler, 4× 5×5 mm magnets for seating, spring-loaded locking blades/tension bolts for the mechanical lock, tool housing (rack) with 90° pick-up | No files. SolidWorks-only and sized for ABB YuMi |
| [gin66/FastAccelStepper](https://github.com/gin66/FastAccelStepper) | MIT | ESP32 step generation (hardware RMT/MCPWM, trapezoid ramps, non-blocking) — firmware dependency | — |
| [teemuatlut/TMCStepper](https://github.com/teemuatlut/TMCStepper) | MIT | TMC2209 UART config (current, microsteps, StealthChop/SpreadCycle, StallGuard read) — firmware dependency | — |
| [rahulsarchive/V-slot-Gantry](https://github.com/rahulsarchive/V-slot-Gantry) | none stated | Idea: one fixed wheel set + one cantilevered/eccentric preload set | Files (no license; designed for 2020) |
| [bdring/FluidNC](https://github.com/bdring/FluidNC), [grblHAL/ESP32](https://github.com/grblHAL/ESP32) | GPL-3.0 | Considered, not used (see below) | — |

## Notes per source

### SO-ARM100 / SO-101
- Servo facts used in the sim: STS3215 12 V = 30 kg·cm stall (README line 73). STS3250 12 V = 50 kg·cm (spec).
- `Simulation/SO101/joints_properties.xml` gives a measured MuJoCo class for the STS3215. We reuse it and scale `forcerange` to each servo's stall torque.
- The SO-101 leader arm (7.4 V STS3215) can drive our J1–J5 + gripper for teleop, because the joint order matches. The gantry X/Z would need a separate input (keyboard/gamepad).

### LeRobot plugin conventions (from `docs/source/integrate_hardware.mdx`)
1. Package name starts with `lerobot_robot_` → ours: `lerobot_robot_gantry_arm`.
2. `GantryArmConfig` / `GantryArm` naming pair; config registered with `@RobotConfig.register_subclass("gantry_arm")`.
3. Config in `config_gantry_arm.py`, robot in `gantry_arm.py`.
4. Both exported from `__init__.py`.
- `supports_position_hold = True` is fine for us: all actions are position targets (gantry included). The driver must return the target it actually applied (after clipping).

### lekiwi_pincopen
- Shows the cleanest way to mix servo models: `replace(motor, model="sts3250")` for joints listed in config, then a single `FeetechMotorsBus`. Our J2 uses **two** STS3250 on one joint. LeRobot has no ganged-motor concept, so we expose one action key `shoulder_lift.pos` and write it to both IDs (2 and 3), with the second servo's direction set by `drive_mode`/calibration.

## Gaps (we have to build these)
1. **Dual-servo shoulder**: no existing LeRobot driver commands two servos as one joint. Risk: the two servos fight each other if calibration differs → watch `Present_Load` of both and flag a mismatch.
2. **Gantry driver**: no LeRobot robot has stepper axes over a serial link to an MCU. New `GantryDriver` class + ESP32 line protocol.
3. **ESP32 firmware**: FluidNC/grblHAL would work for homing and limits, but they speak G-code, are GPL-3.0 and are heavy for 2 axes. The spec asks for a simple line protocol with a testable homing state machine, so we write a small firmware on FastAccelStepper + TMCStepper (both MIT).
4. **Tool coupler with pogo pins for 12 V + TTL bus**: nothing open-source found. RobotToolChanger gives the mechanical concept only. We design our own: 4 magnets + a cam-lock, 4 pogo pins (12 V, GND, DATA, tool-ID).
5. **450 mm reach on hobby bus servos with 500 g payload**: no reference design. SO-101 and PAROL6 are both much shorter or use steppers. The Step 1 torque check decides whether this is feasible.
6. **Self-locking Z: the common T8 screw is NOT self-locking.** The "T8" sold with most printer kits is T8×8 (4-start, 8 mm lead). Lead angle = atan(8 / (π·7 mm)) ≈ 20°. With a brass or POM nut (μ ≈ 0.10–0.15) the friction angle is only 6–9°, so the screw back-drives and the platform **will drop** when the motor loses power.
   | Screw | Lead angle | Self-locking? |
   |---|---|---|
   | T8×8 (4-start) | 20.0° | No |
   | T8×4 (2-start) | 10.3° | No |
   | T8×2 (1-start) | 5.2° | Yes at μ ≥ 0.10, but only just (no vibration margin) |
   The fix is a design choice and is asked as a question before Step 1.
