"""Compile the real firmware core (firmware/lib/gantry_core) + a simulated board into a
shared library and expose it as a serial-like transport for GantryDriver.

This is a contract test across the PC <-> ESP32 seam: the Python driver talks to the
same C++ code that runs on the ESP32. Only the motors/switches are simulated.
"""

from __future__ import annotations

import ctypes
import shutil
import subprocess
from collections import deque
from pathlib import Path

FW = Path(__file__).resolve().parents[3] / "firmware"
BUILD = FW / ".build"


def build_lib() -> Path | None:
    if shutil.which("g++") is None:
        return None
    BUILD.mkdir(exist_ok=True)
    out = BUILD / "libgantry_bridge.so"
    srcs = [FW / "lib/gantry_core/src/gantry_core.cpp", FW / "lib/gantry_core/src/arm_can.cpp", FW / "test/bridge/bridge.cpp"]
    deps = [*srcs, FW / "lib/gantry_core/src/gantry_core.h", FW / "lib/gantry_core/src/arm_can.h", FW / "test/sim_mks.h"]
    if not out.exists() or any(s.stat().st_mtime > out.stat().st_mtime for s in deps):
        subprocess.run(["g++", "-std=c++17", "-O1", "-shared", "-fPIC", "-I", str(FW / "lib/gantry_core/src"), *map(str, srcs), "-o", str(out)], check=True)
    return out


class FirmwareTransport:
    """write()/readline() like pyserial; each exchange advances simulated time by `dt` seconds."""

    def __init__(self, lib_path: Path, dt: float = 0.05):
        self.lib = ctypes.CDLL(str(lib_path))
        self.lib.gb_create.restype = ctypes.c_void_p
        for fn, args in (("gb_destroy", [ctypes.c_void_p]), ("gb_handle", [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]),
                         ("gb_tick", [ctypes.c_void_p, ctypes.c_float, ctypes.c_char_p, ctypes.c_int]),
                         ("gb_set_estop", [ctypes.c_void_p, ctypes.c_int]), ("gb_set_tool_raw", [ctypes.c_void_p, ctypes.c_int]),
                         ("gb_arm_block", [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]), ("gb_arm_unblock", [ctypes.c_void_p, ctypes.c_int])):
            getattr(self.lib, fn).argtypes = args
        self.lib.gb_tick.restype = ctypes.c_int
        self.board = self.lib.gb_create()
        self.dt = dt
        self.rx: deque[bytes] = deque()
        self.sent: list[str] = []

    def close(self):
        if self.board:
            self.lib.gb_destroy(self.board)
            self.board = None

    def tick(self, seconds: float):
        if not self.board:
            return
        buf = ctypes.create_string_buffer(160)
        t = 0.0
        while t < seconds:
            if self.lib.gb_tick(self.board, ctypes.c_float(self.dt), buf, 160):
                self.rx.append(buf.value + b"\n")
            t += self.dt

    def write(self, data: bytes) -> int:
        if not self.board:
            raise OSError("port closed")
        for line in data.decode().splitlines():
            self.tick(self.dt)  # real time passes between commands
            self.sent.append(line)
            out = ctypes.create_string_buffer(160)
            self.lib.gb_handle(self.board, line.encode(), out, 160)
            self.rx.append(out.value + b"\n")
        return len(data)

    def readline(self) -> bytes:
        if not self.board:
            raise OSError("port closed")
        if not self.rx:
            self.tick(self.dt)
        return self.rx.popleft() if self.rx else b""

    def set_estop(self, on: bool):
        self.lib.gb_set_estop(self.board, int(on))

    def set_tool_raw(self, raw: int):
        self.lib.gb_set_tool_raw(self.board, raw)

    def arm_block(self, joint: int, deg: float):
        self.lib.gb_arm_block(self.board, joint, ctypes.c_float(deg))

    def arm_unblock(self, joint: int):
        self.lib.gb_arm_unblock(self.board, joint)
