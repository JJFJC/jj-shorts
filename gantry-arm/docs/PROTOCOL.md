# Gantry controller line protocol (ESP32 ↔ PC)

USB serial at 115200 8N1. One ASCII command per line (`\n`), max 95 characters, case-insensitive.
Every request gets exactly one reply line. Asynchronous events (`ev …`) can arrive between replies.

**Optional checksum:** append `*HH`, the XOR of every byte before `*`, written as two hex digits (NMEA style). For example, `MOVE X100 Z20*` followed by the two hex digits. A bad checksum returns `err CHECKSUM` and nothing moves. The Python driver always sends a checksum.

## Commands

| Command | Reply | Notes |
|---|---|---|
| `PING` | `ok PONG <fw>` | |
| `STATUS` | `st S=<state> X=<mm> Z=<mm> H=<homed> L=<switches> T=<tool>` | state ∈ IDLE, MOVING, HOMING, ESTOP, FAULT. H e.g. `XZ`, `-Z`. L = X-min, X-max, Z-min, Z-max (`-` = open). |
| `HOME [X] [Z]` | `ok` → `ev HOMED Z`, `ev HOMED X` / `ev HOME_FAIL <ax>` | No axis = both, **Z first** (lifts the arm clear before X travels). Seek fast → back off → slow re-approach → offset → logical 0. |
| `MOVE [X<mm>] [Z<mm>] [F<mm/s>]` | `ok` / `err NOT_HOMED X` / `err LIMIT Z` / `err BUSY` | Absolute target. Coordinated: per-axis speeds are scaled so both axes finish together (capped at each axis max). The whole command is rejected if any axis is out of its soft limits. |
| `JOG [X<mm/s>] [Z<mm/s>]` | `ok` | Velocity jog toward the soft limit. Must be refreshed within **300 ms**, or the axis stops with `ev JOG_TIMEOUT`. `0` stops that axis. |
| `STOP` | `ok` | Decelerating stop. Aborts homing (that axis becomes unhomed). |
| `ESTOP` | `ok` | Software e-stop: immediate stop, drivers disabled, latched. |
| `RESET` | `ok` / `err ESTOP_HW` | Clears the latch only once the hardware e-stop is released. X must be re-homed (the belt can move when unpowered). Z keeps its home (the T8×2 self-locks). |
| `LIMITS X<lo>,<hi> Z<lo>,<hi>` | `ok` / `err LIMIT <ax>` | Narrow the soft limits at runtime. They can never be widened past the firmware build limits (X 0–800, Z 0–500). |
| `LED IDLE\|MOVING\|HOMING\|FAULT\|OFF` | `ok` | Request a status colour. Firmware states override it (e-stop/fault = red, homing = amber, moving = green). The PC uses `LED FAULT` for a collision it detects. |
| `TOOL` | `ok TOOL <id>` | Reads the TOOL_ID pogo pin: 0 none, 1 gripper (1 kΩ), 2 suction (2.2 kΩ), 3 pen (4.7 kΩ), 4 phone mount (10 kΩ), −1 shorted. |

Errors: `err EMPTY | TOO_LONG | CHECKSUM | UNKNOWN_CMD | BAD_ARG | ESTOP | ESTOP_HW | NOT_HOMED <ax> | LIMIT <ax> | BUSY`.

Events: `ev BOOT <fw>`, `ev HOMED <ax>`, `ev HOME_FAIL <ax>`, `ev LIMIT_HIT <ax>` (an end switch hit while homed: axis stops and becomes unhomed), `ev ESTOP`, `ev JOG_TIMEOUT`.

## Safety behaviour (tested in `firmware/test/test_core`)

- **Hardware e-stop:** a normally-closed loop, so a broken wire counts as pressed. It stops both axes on the next loop, disables the drivers and latches. It is independent of the PC.
- **Arm servos:** the same e-stop button cuts the 12 V feed to the arm servos through a contactor/relay in the controller box (hardware, not firmware).
- **Moves before homing** are refused.
- **End switches while homed** are treated as over-travel and fault the axis.
- **Over-long or corrupt lines** are discarded whole and never partly executed.
