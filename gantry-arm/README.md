# Gantry Arm

A 3D-printed 5-DOF + gripper robot arm on an X/Z V-slot gantry. **Design and simulation only.** Nothing has been ordered.

| Step | Output |
|---|---|
| 0 Research | `docs/RESEARCH.md` |
| 1 Simulation | `params.py`, `sim/`, `urdf/`, `mjcf/`, `docs/SIM_REPORT.md` |
| 2 CAD | `cad/`, `cad/out/`, `docs/PRINT_LIST.md` |
| 3 Software | `software/lerobot_robot_gantry_arm/`, `firmware/` |
| 4 BOM | `docs/BOM.md` |

```bash
pip install -r requirements.txt
python sim/static_check.py && python sim/report.py   # regenerate URDF, MJCF and SIM_REPORT
pytest tests
```

All dimensions, masses and ratings live in `params.py`.
