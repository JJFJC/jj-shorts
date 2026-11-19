"""LeRobot third-party plugin for the Gantry Arm (robot type "gantry_arm")."""

from .config_gantry_arm import GantryArmConfig
from .gantry_arm import GantryArm

__all__ = ["GantryArmConfig", "GantryArm"]
