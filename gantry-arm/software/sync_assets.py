"""Copy the generated model + geometry from the design repo into the plugin package.

Run after any change to params.py / CAD:  python software/sync_assets.py
The plugin is standalone at runtime (no import of params.py); tests/test_assets_in_sync.py
fails if these copies drift from the design.
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "sim")]
import model as M  # noqa: E402
import params as P  # noqa: E402

ASSETS = Path(__file__).resolve().parent / "lerobot_robot_gantry_arm" / "lerobot_robot_gantry_arm" / "assets"


def geometry() -> dict:
    A = P.ARM
    k = json.loads((ROOT / "sim" / "out" / "results.json").read_text())["k_opt"] if (ROOT / "sim" / "out" / "results.json").exists() else 0.0
    return {
        "x_travel": P.X_TRAVEL, "z_travel": P.Z_TRAVEL,
        "x0": -P.X_TRAVEL / 2,  # world x of the J1 axis at gantry_x = 0 (column centre)
        "j1_y": P.Z_PROFILE[1] / 2 + 12 + P.PLATFORM["offset"],
        "j2_z0": P.X_PROFILE[0] + 10 + P.Z_BOTTOM_CLEAR + A["base_h"],  # J2 height at gantry_z = 0
        "upper": A["upper"], "fore": A["fore"], "tool": A["wrist"] + A["coupler"] + A["tool_tcp"],
        "lateral": -M.wrist_center_y(),  # roll axis offset along -turret_y (world +x at yaw 0)
        "limits_deg": P.JOINT_LIMITS,
        "spring_k_N_per_mm": k,
        "servos": {j: {"model": s.model, "count": n, "stall_Nm": s.stall_Nm, "rated_Nm": s.rated_Nm} for j, (s, n) in P.JOINT_SERVOS.items()},
    }


def main():
    ASSETS.mkdir(parents=True, exist_ok=True)
    g = geometry()
    (ASSETS / "geometry.json").write_text(json.dumps(g, indent=1, sort_keys=True))
    (ASSETS / "gantry_arm.xml").write_text(M.to_mjcf(g["spring_k_N_per_mm"]))
    print("synced", ASSETS)


if __name__ == "__main__":
    main()
