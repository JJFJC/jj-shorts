# Gantry Arm — software

Three pieces:

| Piece | Where | Tests |
|---|---|---|
| ESP32 gantry firmware (homing, soft limits, e-stop, line protocol, LEDs, tool ID) | `firmware/` | 16 native tests: `firmware/test/run_native.sh` (or `pio test -e native`) |
| LeRobot plugin, robot type **`gantry_arm`** | `software/lerobot_robot_gantry_arm/` | 42 tests: `pytest software/lerobot_robot_gantry_arm/tests` |
| Skills + CLI (IK, vision, pick & place, plotter, tool changer) | `…/lerobot_robot_gantry_arm/skills/`, `gantry-arm` command | included above; demos: `python software/demos/sim_demos.py` |

Everything runs **without hardware**: `--robot.backend=sim` (or `gantry-arm --sim …`) swaps the servos and gantry for the MuJoCo model of the same robot. The model uses the CAD masses, the measured servo stiffness and the shoulder spring.

## Install

```bash
pip install -e "software/lerobot_robot_gantry_arm[dev]"   # lerobot, mujoco, opencv, svgpathtools
cd firmware && pio run -e esp32dev -t upload                  # flash the ESP32
```

## What it can do

| Skill | Command | Notes |
|---|---|---|
| Pen plotting | `gantry-arm plot drawing.svg` | SVG → fitted onto an A4 page in the work area. Strokes too wide for one gantry position are split, and the gantry only moves while the pen is up. Sim: pen tip within 0.8 mm of the drawing (`docs/img/demo_plotter.png`). |
| Colour sorting / pick & place | `gantry-arm sort --bins red=330,160 blue=330,220` | Eyes-camera detection → table mm → approach, descend, grip, lift, place. `pick_place.pick_and_place()` for your own scripts. |
| Automatic tool change | `gantry-arm tool 3` | Passive rack (`tool_rack_1/2`): slide into the fork, then the **gantry Z** lifts to break the magnets. The TOOL_ID pogo pin confirms the new tool. |
| Camera calibration | `gantry-arm calibrate-camera --camera 0` | 4 AprilTags (36h11, ids 0–3, 40 mm) at the work-area corners. Synthetic test: 0.5 mm RMS. The calibration follows the X carriage, so the head can move. |
| LeRobot teleop / record / train | `lerobot-record --robot.type=gantry_arm …` | Standard LeRobot flow; the plugin is auto-discovered (`lerobot_robot_` prefix). |
| Collision stop | always on | Load not explained by gravity + tracking error + stalled joint → hold the pose, LEDs red. |

**Work area:** the reliable table rectangle, with the tool pointing down, is **x −450…450 mm, y 110…300 mm** (`kinematics.WORK_AREA`, enforced by a test). Higher up, reach grows to y ≈ 420 mm.

## Robot class (`GantryArm`)

- **Action/observation:** `gantry_x.pos`, `gantry_z.pos` (mm), `base_yaw…wrist_roll.pos` (deg, 0 = arm straight up), `gripper.pos` (0 closed – 100 open), plus camera images.
- **Twin servos (J2, J3):** each joint is one action key. The two servos (IDs 2/3 and 4/5) get the same goal. The mirrored servo's calibration has `drive_mode` flipped. If the pair disagrees by more than 3° for 5 reads, the robot faults.
- **Servo tuning** is written on every connect (lekiwi_pincopen pattern): P = 32 on the heavy joints, 16 elsewhere, and a gripper torque limit.
- **Gravity feed-forward:** each goal is shifted by τ_gravity / k_servo, computed from the CAD-mass model. In the sim, with 500 g at full reach, joint error drops from ~1.2° to under 0.05°. Set `--robot.payload_g` when carrying something heavy.
- **Gantry:** absolute `MOVE` commands to the ESP32. A move is only re-sent when the target changes by more than 0.5 mm. The firmware does the acceleration ramps and enforces the soft limits.

## LeRobot workflow (teach by demonstration)

```bash
# drive it and record episodes (keyboard / gamepad teleop or your own leader device)
lerobot-record --robot.type=gantry_arm --robot.port=/dev/ttyUSB0 --robot.gantry_port=/dev/ttyACM0 \
  --robot.cameras="{wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, head_left: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}" \
  --dataset.repo_id=<you>/gantry_arm_sorting --dataset.single_task="sort the blocks by colour" ...
# then train ACT / SmolVLA with lerobot-train on that dataset
```

An SO-101 leader arm can drive J1 to the gripper, but its joint names differ (`shoulder_pan` vs `base_yaw`, etc.) and it has no gantry axes. That needs a small teleoperator plugin that maps the names and adds keyboard X/Z. **Not built yet.**

## Safety layers

1. **Hardware e-stop:** a normally-closed loop. It cuts the 12 V servo feed and disables the steppers, independent of any software.
2. **Firmware:** e-stop latch, soft limits, over-travel switches, jog watchdog, and refusing moves before homing. All of these are tested.
3. **Plugin:** joint limits, a per-step size limit, twin-servo mismatch, and collision stop. The IK rejects poses that would drive the arm through its own platform.

## Honest status

- **Never run on hardware** (nothing has been bought). The ESP32 `main.cpp` could not be compiled here because the PlatformIO registry is blocked in this environment. The core logic it calls is tested.
- **Values that need checking on the real robot:**
  - The STS `Present_Load` sign and scale (`CollisionMonitor.sign`).
  - The feed-forward gain.
  - The servo stiffness used in the model. It's the SO-101 measurement, scaled to each servo.
- **The sim gripper doesn't physically grasp** (no contact physics). Pick-and-place is verified on motion and timing, not on friction.
