# Gantry Arm

A 3D-printed 5-DOF + gripper robot arm on an X/Z V-slot gantry. **Design and simulation only.** Nothing has been ordered.

| Step | Output |
|---|---|
| 0 Research | `docs/RESEARCH.md` |
| 1 Simulation | `params.py`, `sim/`, `urdf/`, `mjcf/`, `docs/SIM_REPORT.md` |
| 2 CAD + renders | `cad/`, `cad/out/` (STEP + STL), `docs/PRINT_LIST.md`, `docs/DESIGN.md`, `render/out/` |
| 3 Software + skills | `firmware/` (ESP32), `software/lerobot_robot_gantry_arm/` (LeRobot plugin `gantry_arm`, skills, `gantry-arm` CLI), `docs/SOFTWARE.md`, `docs/PROTOCOL.md`, `docs/img/demo_*.png` |
| 2b Inside mechanics | `cad/internals.py`, `cad/servo_pack.py`, `render/render_mechanics.py`, `docs/MECHANICS.md`, `docs/ASSEMBLY.md`, `render/out/mech_*.png` |
| 4 BOM | `docs/BOM.md` (not written yet) |

```bash
pip install -r requirements.txt
python sim/static_check.py && python sim/report.py   # regenerate URDF, MJCF and SIM_REPORT
pytest tests
```

All dimensions, masses and ratings live in `params.py`.

```bash
python cad/build_all.py                    # STEP/STL, mass props, PRINT_LIST (~15 s)
python sim/static_check.py && python sim/report.py
python render/render_blender.py --all      # 8 Cycles renders, 3 CPU threads (~30 min)
```

```bash
firmware/test/run_native.sh                                   # 16 firmware tests (no hardware)
pip install -e "software/lerobot_robot_gantry_arm[dev]"
pytest software/lerobot_robot_gantry_arm/tests                # 42 plugin + skill tests (MuJoCo sim)
gantry-arm --sim plot drawing.svg                             # try a skill on the simulated robot
python software/demos/sim_demos.py                            # regenerate docs/img/demo_*.png
python cad/servo_pack.py && python cad/internals.py            # inside mechanics (servo packing ~2 min, parts + section halves)
python render/render_mechanics.py --view overview             # also: gears, shoulder, wrist, exploded
```
