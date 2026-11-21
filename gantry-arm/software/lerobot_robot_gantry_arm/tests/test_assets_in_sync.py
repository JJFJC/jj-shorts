"""The plugin ships copies of the design model; fail if they drift from params.py / CAD."""

import importlib.util
import json
from pathlib import Path

import pytest

DESIGN = Path(__file__).resolve().parents[3]
SYNC = DESIGN / "software" / "sync_assets.py"


def test_assets_match_design():
    if not (DESIGN / "params.py").exists():
        pytest.skip("installed without the design repo")
    pytest.importorskip("mujoco")
    spec = importlib.util.spec_from_file_location("sync_assets", SYNC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    from lerobot_robot_gantry_arm.kinematics import ASSETS

    assert json.loads((ASSETS / "geometry.json").read_text()) == json.loads(json.dumps(mod.geometry(), sort_keys=True)), "run software/sync_assets.py"
    import model as M

    assert (ASSETS / "gantry_arm.xml").read_text() == M.to_mjcf(mod.geometry()["spring_k_N_per_mm"]), "run software/sync_assets.py"
