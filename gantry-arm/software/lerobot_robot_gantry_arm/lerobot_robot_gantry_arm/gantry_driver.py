"""PC side of the ESP32 gantry line protocol (docs/PROTOCOL.md).

The serial port is the hardware boundary: `GantryDriver` talks to anything with
`write(bytes)` and `readline() -> bytes`. Real use passes a pyserial port; tests pass the
real firmware core compiled as a shared library (see tests/firmware_bridge.py).
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable, Protocol

logger = logging.getLogger(__name__)


class Transport(Protocol):
    def write(self, data: bytes) -> int: ...
    def readline(self) -> bytes: ...


class GantryError(RuntimeError):
    """The controller answered `err ...`."""

    def __init__(self, code: str, line: str):
        super().__init__(line)
        self.code = code


# ---------------------------------------------------------------- framing


def checksum(body: str) -> int:
    c = 0
    for b in body.encode("ascii"):
        c ^= b
    return c


def frame(cmd: str) -> bytes:
    body = cmd.strip()
    return f"{body}*{checksum(body):02X}\n".encode("ascii")


@dataclass
class Status:
    state: str
    x: float
    z: float
    homed_x: bool
    homed_z: bool
    limits: str
    tool: int

    @property
    def homed(self) -> bool:
        return self.homed_x and self.homed_z


def parse_status(line: str) -> Status:
    if not line.startswith("st "):
        raise ValueError(f"not a status line: {line!r}")
    kv = dict(tok.split("=", 1) for tok in line[3:].split())
    return Status(
        state=kv["S"], x=float(kv["X"]), z=float(kv["Z"]),
        homed_x="X" in kv["H"], homed_z="Z" in kv["H"], limits=kv["L"], tool=int(kv["T"]),
    )


@dataclass
class JointState:
    deg: dict
    homed: list
    faults: dict  # joint -> "S" stalled, "C" no reply, "H" homing failed

    @property
    def ok(self) -> bool:
        return not self.faults


def parse_jpos(line: str) -> JointState:
    """'ok JPOS a b c d e H=12-45 F=--S--' -> JointState."""
    t = line.split()
    if len(t) < 9 or t[1] != "JPOS":
        raise GantryError("BAD_REPLY", line)
    names = GantryDriver.ARM_JOINTS
    deg = {n: float(v) for n, v in zip(names, t[2:7])}
    h = t[7].split("=", 1)[1]
    f = t[8].split("=", 1)[1]
    return JointState(deg=deg, homed=[c != "-" for c in h], faults={names[i]: c for i, c in enumerate(f) if c != "-"})


def _fmt(v: float) -> str:
    return f"{v:.3f}"


# ---------------------------------------------------------------- driver


class GantryDriver:
    LED_STATES = ("IDLE", "MOVING", "HOMING", "FAULT", "OFF")

    def __init__(self, transport: Transport, on_event: Callable[[str], None] | None = None, timeout_s: float = 1.0):
        self.t = transport
        self.on_event = on_event
        self.timeout_s = timeout_s
        self.events: list[str] = []

    @classmethod
    def open(cls, port: str, baud: int = 115200, **kw) -> "GantryDriver":
        import serial  # pyserial

        s = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.2)  # ESP32 may reset on open; let it boot
        s.reset_input_buffer()
        return cls(s, **kw)

    # -------------------------------------------------------- low level
    def _readline(self) -> str:
        raw = self.t.readline()
        return raw.decode("ascii", "replace").strip() if raw else ""

    def _event(self, line: str):
        self.events.append(line)
        logger.info("gantry %s", line)
        if self.on_event:
            self.on_event(line)

    def request(self, cmd: str) -> str:
        """Send one command; return its reply line. Events seen meanwhile are dispatched."""
        self.t.write(frame(cmd))
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            line = self._readline()
            if not line:
                continue
            if line.startswith("ev "):
                self._event(line)
                continue
            if line.startswith("err "):
                code = line.split()[1]
                raise GantryError(code, line)
            return line
        raise TimeoutError(f"no reply to {cmd!r}")

    def poll_events(self, max_lines: int = 20) -> list[str]:
        out = []
        for _ in range(max_lines):
            line = self._readline()
            if not line:
                break
            if line.startswith("ev "):
                self._event(line)
                out.append(line)
        return out

    # -------------------------------------------------------- commands
    def ping(self) -> str:
        return self.request("PING").split()[-1]

    def status(self) -> Status:
        return parse_status(self.request("STATUS"))

    def home(self, axes: str = "", wait: bool = True, timeout_s: float = 120.0) -> None:
        self.request(f"HOME {' '.join(axes.upper())}".strip())
        if not wait:
            return
        want = set(axes.upper() or "XZ")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for ev in self.poll_events():
                if ev.startswith("ev HOME_FAIL"):
                    raise GantryError("HOME_FAIL", ev)
            st = self.status()
            if (("X" not in want) or st.homed_x) and (("Z" not in want) or st.homed_z) and st.state != "HOMING":
                return
        raise TimeoutError("homing timed out")

    def move(self, x: float | None = None, z: float | None = None, feed: float | None = None) -> None:
        parts = ["MOVE"]
        if x is not None:
            parts.append(f"X{_fmt(x)}")
        if z is not None:
            parts.append(f"Z{_fmt(z)}")
        if len(parts) == 1:
            return
        if feed is not None:
            parts.append(f"F{_fmt(feed)}")
        self.request(" ".join(parts))

    def jog(self, vx: float = 0.0, vz: float = 0.0) -> None:
        self.request(f"JOG X{_fmt(vx)} Z{_fmt(vz)}")

    def stop(self) -> None:
        self.request("STOP")

    def estop(self) -> None:
        self.request("ESTOP")

    def reset(self) -> None:
        self.request("RESET")

    def set_limits(self, x: tuple[float, float] | None = None, z: tuple[float, float] | None = None) -> None:
        parts = ["LIMITS"]
        if x:
            parts.append(f"X{_fmt(x[0])},{_fmt(x[1])}")
        if z:
            parts.append(f"Z{_fmt(z[0])},{_fmt(z[1])}")
        self.request(" ".join(parts))

    def led(self, state: str) -> None:
        state = state.upper()
        if state not in self.LED_STATES:
            raise ValueError(state)
        self.request(f"LED {state}")

    def tool_id(self) -> int:
        return int(self.request("TOOL").split()[-1])

    # ------------------------------------------------------------------ arm v2 (stepper joints on CAN)
    ARM_JOINTS = ("base_yaw", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")  # J1..J5

    def jhome(self, wait: bool = True, timeout_s: float = 120.0) -> None:
        self.request("JHOME")
        if not wait:
            return
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for ev in self.poll_events():
                if ev.startswith("ev JHOME_FAIL") or ev.startswith("ev JCOMM"):
                    raise GantryError(ev.split()[1], ev)
            pos = self.jpos()
            if all(pos.homed):
                return
        raise TimeoutError("arm homing timed out")

    def jmove(self, joints: dict, speed: float | None = None) -> None:
        """Coordinated joint move (degrees). speed in (0, 1] of the slowest joint's max."""
        parts = ["JMOVE"]
        for i, name in enumerate(self.ARM_JOINTS):
            if name in joints:
                parts.append(f"J{i + 1}{_fmt(joints[name])}")
        if len(parts) == 1:
            return
        if speed is not None:
            parts.append(f"F{_fmt(min(1.0, max(0.01, speed)))}")
        self.request(" ".join(parts))

    def jpos(self) -> "JointState":
        return parse_jpos(self.request("JPOS"))

    def wait_idle(self, timeout_s: float = 60.0) -> Status:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            st = self.status()
            if st.state == "IDLE":
                return st
            if st.state in ("ESTOP", "FAULT"):
                raise GantryError(st.state, f"gantry {st.state}")
        raise TimeoutError("gantry did not settle")
