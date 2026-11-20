from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("gantry_arm")
@dataclass(kw_only=True)
class GantryArmConfig(RobotConfig):
    """Gantry Arm: X/Z V-slot gantry (ESP32) + 5-DOF Feetech arm + quick-change tools.

    CLI example:
        lerobot-teleoperate --robot.type=gantry_arm --robot.port=/dev/ttyUSB0 \\
            --robot.gantry_port=/dev/ttyACM0 --teleop.type=keyboard
    """

    # "hardware" or "sim" (MuJoCo model of the same robot; no hardware needed)
    backend: str = "hardware"
    port: str = "/dev/ttyUSB0"  # Waveshare bus servo adapter (arm)
    gantry_port: str = "/dev/ttyACM0"  # ESP32 USB serial (gantry)

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # safety / motion
    max_relative_target: float | None = 15.0  # deg per action step (gantry: mm)
    gantry_feed_mm_s: float = 150.0
    gantry_deadband_mm: float = 0.5  # don't resend a MOVE for smaller changes
    home_gantry_on_connect: bool = True
    disable_torque_on_disconnect: bool = True

    # servo tuning (lekiwi_pincopen heavy-joint pattern): written on every connect
    heavy_joints: tuple[str, ...] = ("shoulder_lift", "elbow_flex", "wrist_flex")
    heavy_p_coefficient: int = 32
    light_p_coefficient: int = 16
    i_coefficient: int = 0
    d_coefficient: int = 32
    gripper_torque_limit: int = 500  # 50 % of max

    # gravity feed-forward (sim: 37 mm -> <1 mm sag at full reach)
    gravity_feedforward: bool = True
    feedforward_gain: float = 1.0
    payload_g: float = 0.0  # set when you know what the tool is carrying

    # ganged servo pairs (J2, J3)
    ganged_tolerance_deg: float = 3.0
    # geared pairs: motor-side offset that makes the two servos push against each other (backlash/2 + preload).
    # Measure on the real joint: jog the pair apart until Present_Load rises on both, then add ~0.15 deg.
    geared_preload_deg: float = 0.4

    # collision detection from servo load
    collision_detection: bool = True
    collision_threshold_pct: float = 22.0
    collision_cycles: int = 3

    # LED status strip (driven by the ESP32)
    led_status: bool = True
