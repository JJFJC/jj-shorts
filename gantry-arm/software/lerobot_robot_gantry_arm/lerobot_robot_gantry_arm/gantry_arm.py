"""LeRobot robot class for the Gantry Arm.

Action / observation keys (all floats):
    gantry_x.pos, gantry_z.pos              mm (ESP32 steppers)
    base_yaw.pos ... wrist_roll.pos         degrees, 0 = arm straight up (calibration pose)
    gripper.pos                             0 closed .. 100 open
plus one image per configured camera (wrist, head_left/head_right for the stereo eyes).

Calibration pose: arm pointing straight up, gripper half open. The mirrored secondary
servo of each ganged pair gets drive_mode=1 so both servos read the same joint angle.
"""

from __future__ import annotations

import logging
import time
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position

from .config_gantry_arm import GantryArmConfig
from .dynamics import ARM_JOINTS, CollisionMonitor, Ganged, GravityModel, decode_load, feedforward, servo_kp_nm_per_deg
from .kinematics import Geometry

logger = logging.getLogger(__name__)

GANTRY = ["gantry_x", "gantry_z"]
JOINTS = GANTRY + ARM_JOINTS + ["gripper"]
MOTORS = [  # name, id, model
    ("base_yaw", 1, "sts3215"),
    ("shoulder_lift", 2, "sts3250"),
    ("shoulder_lift_2", 3, "sts3250"),
    ("elbow_flex", 4, "sts3250"),
    ("elbow_flex_2", 5, "sts3250"),
    ("wrist_flex", 6, "sts3215"),
    ("wrist_roll", 7, "sts3215"),
    ("gripper", 8, "sts3215"),
]


class CollisionError(RuntimeError):
    pass


class GantryArm(Robot):
    config_class = GantryArmConfig
    name = "gantry_arm"
    supports_position_hold = True

    def __init__(self, config: GantryArmConfig):
        super().__init__(config)
        self.config = config
        self.geo = Geometry.load()
        self.ganged = Ganged(tolerance_deg=config.ganged_tolerance_deg)
        self.collision = CollisionMonitor(config.collision_threshold_pct, config.collision_cycles)
        self.kp = {}
        for j in ARM_JOINTS:
            s = self.geo_servo(j)
            p = config.heavy_p_coefficient if j in config.heavy_joints else config.light_p_coefficient
            self.kp[j] = servo_kp_nm_per_deg(s["stall_Nm"], s["count"], p)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.gravity: GravityModel | None = None
        self.bus = None
        self.gantry = None
        self.sim = None
        self._connected = False
        self._last_gantry_cmd: dict | None = None
        self._last_cmd: dict = {}  # last arm goals actually commanded (incl. feed-forward)
        self._last_read: tuple[float, dict] | None = None
        self.collided_joint: str | None = None
        if config.backend == "hardware":
            from lerobot.motors import Motor, MotorNormMode
            from lerobot.motors.feetech import FeetechMotorsBus

            motors = {
                n: Motor(i, model, MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES)
                for n, i, model in MOTORS
            }
            self.bus = FeetechMotorsBus(port=config.port, motors=motors, calibration=self.calibration)
        elif config.backend != "sim":
            raise ValueError(f"backend must be 'hardware' or 'sim', got {config.backend!r}")

    def geo_servo(self, joint: str) -> dict:
        import json

        from .kinematics import ASSETS

        return json.loads((ASSETS / "geometry.json").read_text())["servos"][joint]

    # ------------------------------------------------------------------ features
    @cached_property
    def observation_features(self) -> dict:
        ft = {f"{j}.pos": float for j in JOINTS}
        for cam, cfg in self.config.cameras.items():
            ft[cam] = (cfg.height, cfg.width, 3)
        return ft

    @cached_property
    def action_features(self) -> dict:
        return {f"{j}.pos": float for j in JOINTS}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True if self.bus is None else self.bus.is_calibrated

    # ------------------------------------------------------------------ lifecycle
    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            raise RuntimeError(f"{self} already connected")
        if self.config.gravity_feedforward or self.config.collision_detection or self.config.backend == "sim":
            self.gravity = GravityModel(payload_g=self.config.payload_g)
        if self.config.backend == "sim":
            from .sim_backend import SimBackend

            c = self.config
            p_scale = {j: (c.heavy_p_coefficient if j in c.heavy_joints else c.light_p_coefficient) / 16.0 for j in ARM_JOINTS}
            self.sim = SimBackend(payload_g=c.payload_g, feed_mm_s=c.gantry_feed_mm_s, p_scale=p_scale)
            self.sim.connect()
        else:
            from .gantry_driver import GantryDriver

            self.bus.connect()
            if not self.is_calibrated and calibrate:
                self.calibrate()
            self.configure()
            self.gantry = GantryDriver.open(self.config.gantry_port)
            logger.info("gantry firmware %s", self.gantry.ping())
            if self.config.home_gantry_on_connect:
                self.gantry.home()
        for cam in self.cameras.values():
            cam.connect()
        self._connected = True
        self._led("IDLE")
        logger.info("%s connected (%s)", self, self.config.backend)

    def calibrate(self) -> None:
        if self.bus is None:
            return
        if self._keep_calibration_file(self.bus):
            return
        input("Put the arm STRAIGHT UP (calibration pose = joint zero), gripper half open, then press Enter")
        self.calibration = self.bus.record_calibration("position", full_turn_motors=["wrist_roll"])
        for secondary in self.ganged.pairs.values():  # mirrored twin servo reads the same angle
            self.calibration[secondary].drive_mode = 1 - self.calibration[secondary].drive_mode
        self.bus.write_calibration(self.calibration)
        self._save_calibration()

    def configure(self) -> None:
        if self.bus is None:
            return
        c = self.config
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.set_operating_mode("position", motor)
                joint = motor.removesuffix("_2")
                p = c.heavy_p_coefficient if joint in c.heavy_joints else c.light_p_coefficient
                self.bus.write("P_Coefficient", motor, p)
                self.bus.write("I_Coefficient", motor, c.i_coefficient)
                self.bus.write("D_Coefficient", motor, c.d_coefficient)
                if motor == "gripper":
                    self.bus.write("Max_Torque_Limit", motor, c.gripper_torque_limit)
                    self.bus.write("Protection_Current", motor, 250)
                    self.bus.write("Overload_Torque", motor, 25)

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._led("OFF")
        if self.bus is not None:
            self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
        self._connected = False

    # ------------------------------------------------------------------ io helpers
    def _led(self, state: str):
        if self.config.led_status and self.gantry is not None:
            try:
                self.gantry.led(state)
            except Exception as e:  # LEDs are cosmetic; never fail a motion on them
                logger.debug("led: %s", e)

    def _read_arm(self) -> dict:
        if self.sim is not None:
            return self.sim.read_arm()
        raw = self.bus.sync_read("Present_Position")
        faults = self.ganged.check(raw)
        if faults:
            self._fault(f"ganged servos disagree on {faults}")
        return self.ganged.merge(raw)

    def _read_loads(self) -> dict:
        if self.sim is not None:
            return self.sim.read_loads_pct()
        raw = self.bus.sync_read("Present_Load", normalize=False)
        out = {}
        for j in ARM_JOINTS:
            vals = [decode_load(raw[m]) for m in (j, self.ganged.pairs.get(j)) if m and m in raw]
            out[j] = sum(vals) / len(vals)
        return out

    def _read_gantry(self) -> dict:
        if self.sim is not None:
            return self.sim.gantry_pos()
        st = self.gantry.status()
        return {"gantry_x": st.x, "gantry_z": st.z}

    def _fault(self, why: str):
        logger.error("FAULT: %s", why)
        self._led("FAULT")
        self.hold()
        raise CollisionError(why)

    # ------------------------------------------------------------------ main API
    def get_observation(self) -> dict:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected")
        obs = {f"{j}.pos": v for j, v in {**self._read_gantry(), **self._read_arm()}.items()}
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()
        return obs

    def send_action(self, action: dict) -> dict:
        if not self._connected:
            raise RuntimeError(f"{self} is not connected")
        goal = {k.removesuffix(".pos"): float(v) for k, v in action.items() if k.endswith(".pos")}
        c = self.config
        # joint limits from the design (the firmware enforces the gantry ones as well)
        for j, (lo, hi) in self.geo.limits_deg.items():
            if j in goal:
                goal[j] = min(max(goal[j], lo), hi)
        if "gantry_x" in goal:
            goal["gantry_x"] = min(max(goal["gantry_x"], 0.0), self.geo.x_travel)
        if "gantry_z" in goal:
            goal["gantry_z"] = min(max(goal["gantry_z"], 0.0), self.geo.z_travel)
        present_arm = self._read_arm()
        if c.max_relative_target is not None:
            # arm joints only: the gantry firmware ramps its own moves inside its soft limits
            arm_keys = {k: (v, present_arm[k]) for k, v in goal.items() if k in present_arm and k != "gripper"}
            goal.update(ensure_safe_goal_position(arm_keys, c.max_relative_target))

        # collision check on the measured state before commanding anything new
        now = self._now()
        speed = {}
        if self._last_read is not None and now > self._last_read[0]:
            dt = now - self._last_read[0]
            speed = {j: (present_arm[j] - self._last_read[1].get(j, present_arm[j])) / dt for j in ARM_JOINTS}
        self._last_read = (now, present_arm)
        if c.collision_detection and self.gravity is not None and self._last_cmd and speed:
            tau = self.gravity.torques(present_arm)
            expected = {j: 100.0 * tau[j] / self._stall(j) for j in ARM_JOINTS}
            err = {j: self._last_cmd[j] - present_arm[j] - (expected[j] * self._stall(j) / 100.0) / self.kp[j]
                   for j in ARM_JOINTS if j in self._last_cmd}
            hit = self.collision.update(expected, self._read_loads(), err, speed)
            if hit:
                self.collided_joint = hit
                self._fault(f"collision detected at {hit}")

        # gantry: absolute MOVE, only when the target really changed
        g = {k: goal[k] for k in GANTRY if k in goal}
        if g:
            changed = self._last_gantry_cmd is None or any(abs(g[k] - self._last_gantry_cmd.get(k, 1e9)) > c.gantry_deadband_mm for k in g)
            if changed:
                if self.sim is not None:
                    self.sim.gantry_move(g.get("gantry_x"), g.get("gantry_z"))
                else:
                    self.gantry.move(x=g.get("gantry_x"), z=g.get("gantry_z"), feed=c.gantry_feed_mm_s)
                self._last_gantry_cmd = {**(self._last_gantry_cmd or {}), **g}

        # arm: gravity feed-forward on the goal, then expand ganged pairs
        arm_goal = {k: v for k, v in goal.items() if k in ARM_JOINTS or k == "gripper"}
        cmd = dict(arm_goal)
        if c.gravity_feedforward and self.gravity is not None:
            full = {**present_arm, **arm_goal}
            cmd = feedforward(arm_goal, self.gravity.torques(full), self.kp, c.feedforward_gain)
        if self.sim is not None:
            self.sim.write_arm(cmd)
        elif cmd:
            self.bus.sync_write("Goal_Position", self.ganged.expand(cmd))
        self._last_cmd.update(cmd)
        return {f"{k}.pos": v for k, v in goal.items()}

    def _now(self) -> float:
        return float(self.sim.d.time) if self.sim is not None else time.monotonic()

    def _stall(self, j: str) -> float:
        s = self.geo_servo(j)
        return s["stall_Nm"] * s["count"]

    def hold(self) -> None:
        """Freeze at the measured pose (used on faults and by rollout hold)."""
        if self.sim is not None:
            arm = self.sim.read_arm()
            self.sim.write_arm({k: v for k, v in arm.items() if k != "gripper"})
            p = self.sim.gantry_pos()
            self.sim.gantry_move(p["gantry_x"], p["gantry_z"])
            return
        if self.bus is not None:
            present = self.bus.sync_read("Present_Position")
            present.pop("gripper", None)
            self.bus.sync_write("Goal_Position", present)
        if self.gantry is not None:
            self.gantry.stop()

    # ------------------------------------------------------------------ helpers for skills
    def step(self, seconds: float):
        """Let time pass: real sleep on hardware, physics steps in sim."""
        if self.sim is not None:
            self.sim.step(seconds)
        else:
            time.sleep(seconds)

    def tool_id(self) -> int:
        return self.sim.tool_id if self.sim is not None else self.gantry.tool_id()
