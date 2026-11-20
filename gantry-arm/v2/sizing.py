"""Arm v2 sizing: closed-loop NEMA steppers + precision planetary gearboxes (owner choice 2026-11-20).

    python v2/sizing.py   -> v2/out/sizing.json + docs/ARM_V2.md

Same kinematics as the servo arm (J2->TCP 450 mm), payload target raised from 0.5 to 1.0 kg at full reach.
Everything is a lumped-mass hand calculation at the worst pose (arm horizontal, full reach), plus
acceleration torque. Motor/gearbox figures are typical catalogue CLASS values, marked VERIFY: replace them
with the chosen part's datasheet torque curve before buying.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
import params as P  # noqa: E402

G = 9.81
A = P.ARM
L_UP, L_FO = A["upper"], A["fore"]
L_WR = A["wrist"] + A["coupler"] + A["tool_tcp"]  # J4 -> TCP
REACH = L_UP + L_FO + L_WR  # J2 -> TCP

# ----------------------------------------------------------------------------- catalogue classes (VERIFY)
# holding = rated holding torque; usable = torque left at ~300-600 rpm on 24 V with a closed-loop driver
MOTORS = {
    "NEMA14-34": dict(hold=0.18, usable=0.11, mass=0.22, rpm=600),
    "NEMA17-40": dict(hold=0.42, usable=0.26, mass=0.28, rpm=600),
    "NEMA17-48": dict(hold=0.55, usable=0.32, mass=0.36, rpm=600),
    "NEMA17-60": dict(hold=0.65, usable=0.40, mass=0.48, rpm=600),
    "NEMA23-56": dict(hold=1.26, usable=0.85, mass=0.70, rpm=500),
    "NEMA23-76": dict(hold=1.90, usable=1.25, mass=1.00, rpm=500),
}
GEARBOX = {  # precision planetary (stages 0 = no gearbox): efficiency, lost motion (arcmin), mass (kg), torsional stiffness (N·m/arcmin)
    ("NEMA17", 0): dict(eff=1.0, arcmin=0.0, mass=0.0, stiff=50.0),
    ("NEMA17", 1): dict(eff=0.90, arcmin=5.0, mass=0.25, stiff=0.6),
    ("NEMA17", 2): dict(eff=0.81, arcmin=7.0, mass=0.35, stiff=0.6),
    ("NEMA23", 2): dict(eff=0.81, arcmin=5.0, mass=0.65, stiff=1.5),
    ("NEMA14", 1): dict(eff=0.90, arcmin=10.0, mass=0.12, stiff=0.2),
}
PAYLOAD = 1.0  # kg at the TCP
SF = 1.3  # margin on (static + acceleration) torque
ALPHA = {"base_yaw": 3.0, "shoulder_lift": 5.0, "elbow_flex": 5.0, "wrist_flex": 10.0, "wrist_roll": 10.0}  # rad/s^2

# ----------------------------------------------------------------------------- drive choice per joint
DRIVES = {
    # joint: motor, gearbox stages, gear ratio, extra belt ratio, where the drive sits
    "base_yaw": dict(motor="NEMA17-48", stages=0, ratio=1.0, belt=10.0, where="under the platform, GT2 belt 20T -> 200T ring on the turret (no gearbox, no gear backlash)"),
    "shoulder_lift": dict(motor="NEMA23-76", stages=2, ratio=20.0, belt=1.0, where="in the turret, output flange on the J2 axis"),
    "elbow_flex": dict(motor="NEMA17-60", stages=2, ratio=25.0, belt=1.0, where="at the elbow, coaxial"),
    "wrist_flex": dict(motor="NEMA17-40", stages=2, ratio=15.0, belt=1.0, where="behind the elbow in the forearm, GT2 belt 1:1 to the wrist"),
    "wrist_roll": dict(motor="NEMA14-34", stages=1, ratio=5.0, belt=1.0, where="in the wrist, coaxial with the roll"),
}

# ----------------------------------------------------------------------------- lumped masses (kg), distance along the arm from J2 (mm)


def drive_mass(j):
    d = DRIVES[j]
    m = MOTORS[d["motor"]]
    gb = GEARBOX[(d["motor"].split("-")[0], d["stages"])]
    return m["mass"] + gb["mass"] + (0.08 if d["belt"] > 1 else 0.0)


def arm_parts():
    """(name, mass kg, distance from J2 along the arm mm). Drives on the turret don't load J2."""
    e = L_UP  # elbow
    w = L_UP + L_FO  # wrist pivot (J4)
    return [
        ("upper link (aluminium tube + printed covers)", 0.45, L_UP / 2),
        ("J3 drive", drive_mass("elbow_flex") + 0.10, e),  # + printed housing
        ("J4 drive (behind the elbow)", drive_mass("wrist_flex"), e - 30.0),
        ("forearm link", 0.35, e + L_FO / 2),
        ("wrist housing", 0.20, w + 15.0),
        ("J5 drive", drive_mass("wrist_roll"), w + 20.0),
        ("coupler + gripper (STS3215 gripper)", 0.35, REACH - 30.0),
        ("payload", PAYLOAD, REACH),
    ]


def joint_load(pivot_mm, parts):
    """Static torque and inertia about a pivot for the parts beyond it (arm horizontal)."""
    tau = sum(m * G * (d - pivot_mm) / 1000 for _, m, d in parts if d > pivot_mm - 40)
    J = sum(m * ((d - pivot_mm) / 1000) ** 2 for _, m, d in parts if d > pivot_mm - 40)
    return tau, J


def capacity(j):
    d = DRIVES[j]
    m = MOTORS[d["motor"]]
    gb = GEARBOX[(d["motor"].split("-")[0], d["stages"])]
    n = d["ratio"] * d["belt"]
    belt_eff = 0.97 if d["belt"] > 1 else 1.0
    return dict(usable=m["usable"] * n * gb["eff"] * belt_eff, holding=m["hold"] * n * gb["eff"] * belt_eff, n=n,
                speed_dps=m["rpm"] / n * 6.0, arcmin=gb["arcmin"], stiff=gb["stiff"] * 3437.7 * (d["belt"] ** 2 if d["belt"] > 1 else 1.0))


def size():
    parts = arm_parts()
    out = {"parts": parts, "joints": {}}
    pivots = {"shoulder_lift": 0.0, "elbow_flex": L_UP, "wrist_flex": L_UP + L_FO}
    for j, p in pivots.items():
        tau, J = joint_load(p, parts)
        need = SF * (tau + J * ALPHA[j])
        out["joints"][j] = dict(static=tau, inertia=J, accel=J * ALPHA[j], need=need, **capacity(j))
    # roll: payload 15 mm off the roll axis + roll inertia
    tau5 = PAYLOAD * G * P.MASS["payload_offset"] / 1000
    J5 = PAYLOAD * 0.04**2 / 2 + 0.35 * 0.03**2
    out["joints"]["wrist_roll"] = dict(static=tau5, inertia=J5, accel=J5 * ALPHA["wrist_roll"], need=SF * (tau5 + J5 * ALPHA["wrist_roll"]), **capacity("wrist_roll"))
    # base: no gravity torque (vertical axis); inertia of the whole arm about J1 (J2 sits on the J1 axis)
    _, J1 = joint_load(0.0, parts)
    out["joints"]["base_yaw"] = dict(static=0.0, inertia=J1, accel=J1 * ALPHA["base_yaw"], need=SF * (J1 * ALPHA["base_yaw"] + 0.3), **capacity("base_yaw"))
    for j, r in out["joints"].items():
        r["margin"] = r["usable"] / r["need"]
        r["sag_mm"] = r["static"] / r["stiff"] * ({"shoulder_lift": REACH, "elbow_flex": REACH - L_UP, "wrist_flex": L_WR}.get(j, 0.0))
    # repeatability budget: hysteresis ~20% of the gearbox lost motion when gravity keeps one flank loaded
    lever = {"base_yaw": REACH, "shoulder_lift": REACH, "elbow_flex": REACH - L_UP, "wrist_flex": L_WR, "wrist_roll": 15.0}
    rep = {}
    for j, r in out["joints"].items():
        hyst_rad = math.radians(0.2 * r["arcmin"] / 60.0)
        enc_rad = math.radians(360.0 / 16384 / r["n"])  # 14-bit encoder on the motor shaft
        belt_rad = math.radians(0.02) if DRIVES[j]["belt"] > 1 or j == "wrist_flex" else 0.0
        rep[j] = (hyst_rad + enc_rad + belt_rad) * lever[j]
    out["repeat_mm"] = rep
    out["repeat_rss_mm"] = math.sqrt(sum(v * v for v in rep.values()))
    out["repeat_rss_3arcmin_mm"] = math.sqrt(sum(((math.radians(0.2 * 3 / 60) + (math.radians(0.02) if (DRIVES[j]["belt"] > 1 or j == "wrist_flex") else 0)) * lever[j]) ** 2 for j in rep))
    out["accuracy_backlash_mm"] = sum(math.radians(r["arcmin"] / 60) * lever[j] for j, r in out["joints"].items() if j != "wrist_roll")
    # mass hanging on the Z carriage
    turret_drives = drive_mass("base_yaw") + drive_mass("shoulder_lift")
    arm_m = sum(m for _, m, _ in parts)
    out["mass"] = dict(arm_moving=arm_m, turret_and_drives=turret_drives + 0.6, total_on_z=arm_m + turret_drives + 0.6 + 0.37)
    out["gantry"] = gantry(out)
    out["gantry_rails"] = gantry_rails(out)
    return out


def gantry(out):
    """Re-run the servo-arm gantry checks with the v2 mass (same formulas as sim/static_check.structure)."""
    m_hang = out["mass"]["total_on_z"]
    off = P.PLATFORM["offset"]
    # CoM lever from the column: turret + drives at the J1 axis, the arm parts out along the reach
    near = out["mass"]["turret_and_drives"] + 0.37
    far = sum(m * (off + d) for _, m, d in out["parts"])
    lever = (near * off * 0.8 + far) / m_hang / 1000  # m
    W = m_hang * G
    M = W * lever
    E = P.AL_E * 1e6
    zc = P.Z_CARRIAGE
    h = (P.Z_BOTTOM_CLEAR + P.Z_TRAVEL) / 1000
    tcp_lever = (off + REACH) / 1000
    res = {"m_hang_kg": m_hang, "lever_mm": lever * 1000, "M_col_Nm": M}
    I_s, I_w = P.I_2040["strong"] * 1e-12, P.I_2040["weak"] * 1e-12
    res["col_tcp_mm"] = M * h / (E * I_s) * tcp_lever * 1000
    pitch, n = zc["row_pitch"] / 1000, zc["wheels_per_row"]
    s_y = (P.X_PROFILE[1] + 10) / 1000
    n_side = P.X_CARRIAGE["wheels"] // 2
    for tag, kw in zip(("lo", "hi"), P.WHEEL_K_N_PER_MM):
        k = kw * 1000
        res[f"zc_tcp_mm_{tag}"] = 2 * (M / pitch) / (n * k) / pitch * tcp_lever * 1000
        res[f"xc_tcp_mm_{tag}"] = 2 * (M / s_y) / (n_side * k) / s_y * tcp_lever * 1000
    res["zc_wheel_N"] = M / pitch / n + W / (2 * n)
    res["xc_wheel_N"] = M / s_y / n_side + (m_hang + 1.5) * G / (2 * n_side)
    # first wobble mode (column + carriage rocking), as in static_check
    k_col_x = 3 * E * I_w / h**3
    pitch_x = P.X_CARRIAGE["wheel_pitch"] / 1000
    for tag, kw in zip(("lo", "hi"), P.WHEEL_K_N_PER_MM):
        k = kw * 1000
        k_rot_y = 2 * n_side * k * (pitch_x / 2) ** 2
        kx = 1 / (1 / k_col_x + h**2 / k_rot_y)
        res[f"fx_hz_{tag}"] = math.sqrt(kx / (m_hang + 0.2)) / (2 * math.pi)
    return res


# linear-rail gantry (owner choice 2026-11-20). Class values, VERIFY against the chosen rail/profile datasheets.
GANTRY_V2 = dict(
    z_column="4080 V-slot/T-slot, 80 mm side facing the arm (strong axis)", I_strong=8.4e5, I_weak=2.0e5,  # mm^4
    z_rails="1x HGR15 on the 40 mm face (two rails don't fit), 2 blocks, 90 mm block pitch", z_pitch=90.0, z_span=0.0,
    x_rails="2x HGR15 on a 4080 base (or a flat plate), 80 mm apart, 2 blocks each, 120 mm block pitch", x_pitch=120.0, x_span=80.0,
    block_k=60000.0,  # N/mm per block, radial; miniature/compact guides are tens of N/um (vs V-wheels 0.3-1.5 N/um)
)


def gantry_rails(out):
    g = GANTRY_V2
    m_hang = out["mass"]["total_on_z"] + 0.6  # heavier Z carriage plate + 2 extra blocks
    lever = out["gantry"]["lever_mm"] / 1000
    W = m_hang * G
    M = W * lever
    E = P.AL_E * 1e6
    h = (P.Z_BOTTOM_CLEAR + P.Z_TRAVEL) / 1000
    tcp_lever = (P.PLATFORM["offset"] + REACH) / 1000
    k = g["block_k"] * 1000
    res = {"m_hang_kg": m_hang, "M_col_Nm": M}
    res["col_tcp_mm"] = M * h / (E * g["I_strong"] * 1e-12) * tcp_lever * 1000
    # Z carriage: one rail, 2 blocks: the moment is a couple over the block pitch
    d = (M / (g["z_pitch"] / 1000)) / k
    res["zc_tcp_mm"] = 2 * d / (g["z_pitch"] / 1000) * tcp_lever * 1000
    # X carriage: moment about X taken by the two rails' span
    d = (M / (g["x_span"] / 1000)) / (2 * k)
    res["xc_tcp_mm"] = 2 * d / (g["x_span"] / 1000) * tcp_lever * 1000
    k_col_x = 3 * E * g["I_weak"] * 1e-12 / h**3
    k_rot = 4 * k * (g["x_pitch"] / 2000) ** 2
    kx = 1 / (1 / k_col_x + h**2 / k_rot)
    res["fx_hz"] = math.sqrt(kx / (m_hang + 0.5)) / (2 * math.pi)
    res["total_tcp_mm"] = res["col_tcp_mm"] + res["zc_tcp_mm"] + res["xc_tcp_mm"]
    return res


def report(r):
    L = []
    w = L.append
    w("# Arm v2: closed-loop steppers + precision planetary gearboxes")
    w("")
    w("Owner's choice (2026-11-20), after rejecting the servo arm as not precise or strong enough. Generated by `v2/sizing.py`.")
    w("These are hand calculations with **catalogue class values** for the motors and gearboxes (marked VERIFY). They pick the size of each part, not the exact part.")
    w("")
    w(f"**Targets:** same kinematics (J2→TCP {REACH:.0f} mm), payload **{PAYLOAD:.1f} kg at full reach** (the servo arm managed 0.5 kg), "
      "repeatability in the class of PAROL6 (0.1 mm, official spec) and AR4 (±0.2 mm).")
    w("")
    w("## Drive per joint")
    w("")
    w("| Joint | Motor (closed-loop, 24 V) | Gearbox | Total ratio | Static N·m | Accel N·m | Needed (×1.3) N·m | Usable N·m | Margin | Speed °/s | Where |")
    w("|---|---|---|---|---|---|---|---|---|---|---|")
    names = {"base_yaw": "J1 base yaw", "shoulder_lift": "J2 shoulder", "elbow_flex": "J3 elbow", "wrist_flex": "J4 wrist pitch", "wrist_roll": "J5 wrist roll"}
    for j in ("base_yaw", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
        x, d = r["joints"][j], DRIVES[j]
        gb = (f"{d['ratio']:g}:1 planetary ({d['stages']}-stage)" if d["stages"] else "none") + (f" + belt {d['belt']:g}:1" if d["belt"] > 1 else "")
        flag = "✓" if x["margin"] >= 1.0 else "**✗**"
        w(f"| {names[j]} | {d['motor']} | {gb} | {x['n']:g}:1 | {x['static']:.2f} | {x['accel']:.2f} | {x['need']:.2f} | {x['usable']:.2f} | {x['margin']:.2f} {flag} | {x['speed_dps']:.0f} | {d['where']} |")
    w("")
    w("The gripper stays an STS3215 bus servo. It doesn't need precision, and its finger force is unchanged.")
    w("")
    w("## Mass")
    w("")
    m = r["mass"]
    w("| Item | kg | Distance from J2 (mm) |")
    w("|---|---|---|")
    for n_, kg, d in r["parts"]:
        w(f"| {n_} | {kg:.2f} | {d:.0f} |")
    w(f"| **Moving arm (beyond J2)** | **{m['arm_moving']:.2f}** | |")
    w(f"| Turret + J1/J2 drives | {m['turret_and_drives']:.2f} | at the J1 axis |")
    w(f"| **Total on the Z carriage** | **{m['total_on_z']:.1f}** | servo arm: 2.6 kg |")
    w("")
    w("## Precision budget")
    w("")
    w("- **Repeatability:** the same pose approached the same way, with gravity keeping each gearbox on one tooth flank.")
    w("  - Per joint it adds the 14-bit motor encoder (16384 counts/rev ÷ ratio), about 20% of the gearbox lost motion as hysteresis, and 0.02° per belt.")
    w("- **Accuracy:** hitting an absolute coordinate. It needs calibration, and full backlash shows up whenever a joint reverses against gravity.")
    w("")
    w("| Joint | Lever to TCP (mm) | Repeatability contribution (mm) |")
    w("|---|---|---|")
    for j, v in r["repeat_mm"].items():
        w(f"| {names[j]} | | {v:.3f} |")
    w(f"| **Total (RSS)** | | **{r['repeat_rss_mm']:.2f} mm** with these gearboxes; **{r['repeat_rss_3arcmin_mm']:.2f} mm** with ≤3 arcmin gearboxes on J1–J3 |")
    w("")
    w(f"Worst-case backlash error if every joint reverses at once: {r['accuracy_backlash_mm']:.1f} mm. Avoid it by approaching poses from one direction and keeping gravity on one flank.")
    w("**The gearbox lost-motion spec is the number that decides precision. Buy ≤5 arcmin (precision grade), not the 15–30 arcmin generic ones.**")
    w("")
    w("Static droop from gearbox torsional stiffness (repeatable, so it can be calibrated or fed forward):")
    for j in ("shoulder_lift", "elbow_flex", "wrist_flex"):
        w(f"- {names[j]}: {r['joints'][j]['sag_mm']:.2f} mm at the TCP")
    w("")
    w("## Gantry with the heavier arm")
    w("")
    g = r["gantry"]
    w(f"Load on the Z carriage: **{g['m_hang_kg']:.1f} kg** (servo arm 2.6 kg). Moment on the column: **{g['M_col_Nm']:.1f} N·m** (servo arm 7.5 N·m).")
    w("")
    w("| Check | Servo arm | Arm v2 on the same gantry |")
    w("|---|---|---|")
    w(f"| Column bending at the TCP | 0.9 mm | {g['col_tcp_mm']:.1f} mm |")
    w(f"| Z carriage V-wheel rocking at the TCP | 0.2–0.9 mm | {g['zc_tcp_mm_hi']:.1f}–{g['zc_tcp_mm_lo']:.1f} mm |")
    w(f"| X carriage V-wheel rocking at the TCP | 0.4–2.0 mm | {g['xc_tcp_mm_hi']:.1f}–{g['xc_tcp_mm_lo']:.1f} mm |")
    w(f"| Z wheel load / X wheel load | 24 N / 42 N | {g['zc_wheel_N']:.0f} N / {g['xc_wheel_N']:.0f} N |")
    w(f"| First wobble mode | 6–8 Hz | {g['fx_hz_lo']:.1f}–{g['fx_hz_hi']:.1f} Hz |")
    w("")
    w("On the V-wheel gantry the whole arm would move by millimetres and wobble at a few Hz, wasting the arm's precision.")
    w("")
    w("### Linear-rail gantry (owner's choice, 2026-11-20)")
    w("")
    gr, G2 = r["gantry_rails"], GANTRY_V2
    w(f"- Z column: {G2['z_column']}")
    w(f"- Z guides: {G2['z_rails']}")
    w(f"- X guides: {G2['x_rails']}")
    w("- The figures below are class values: VERIFY the profile inertia and the block stiffness on the chosen parts' datasheets.")
    w("")
    w("| Check | V-wheel gantry | Linear rails |")
    w("|---|---|---|")
    w(f"| Column bending at the TCP | {g['col_tcp_mm']:.1f} mm | {gr['col_tcp_mm']:.2f} mm |")
    w(f"| Z carriage rocking at the TCP | up to {g['zc_tcp_mm_lo']:.1f} mm | {gr['zc_tcp_mm']:.3f} mm |")
    w(f"| X carriage rocking at the TCP | up to {g['xc_tcp_mm_lo']:.1f} mm | {gr['xc_tcp_mm']:.3f} mm |")
    w(f"| **Total gantry sag at the TCP** | **{g['col_tcp_mm'] + g['zc_tcp_mm_lo'] + g['xc_tcp_mm_lo']:.1f} mm** | **{gr['total_tcp_mm']:.2f} mm** (static and repeatable, so calibration takes it out) |")
    w(f"| First wobble mode | {g['fx_hz_lo']:.1f} Hz | {gr['fx_hz']:.0f} Hz |")
    w("")
    w("The gantry is now stiffer than the arm. What's left is the arm's own repeatability.")
    w("")
    w("## CAD (`v2/cad_v2.py`, renders `v2/out/v2_*.png` from `v2/render_v2.py`)")
    w("")
    w("Same kinematics and body frames as the servo arm, so the IK, the tool changer and the workshop layout still apply. 56 parts, zero clashes with the arm straight (checked).")
    w("")
    w("| Joint | Layout |")
    w("|---|---|")
    w("| J1 | NEMA17 under the platform, 20T pulley → 200T belt ring on the turret. Two 61812 bearings on a fixed spigot carry the moment. |")
    w("| J2 | NEMA23-76 + 20:1 planetary lying along the J2 axis inside the turret capsule; the gearbox output flange carries the upper-arm hub. |")
    w("| J3 | NEMA17-60 + 25:1 planetary coaxial at the elbow, its back end inside the upper link. The upper link has a 20×20 aluminium spine under a printed cover. |")
    w("| J4 | NEMA17-40 + 15:1 in a pod 72 mm behind the elbow, where it also counterweights. A 30T/30T belt runs inside the forearm link to the wrist shaft (2× 6801 bearings). |")
    w("| J5 | NEMA14 + 5:1 coaxial in the roll housing, with a 6805 at the flange. |")
    w("| Gantry | X: 2× HGR15 rails on a 10 mm plate, 4 blocks. Z: 4080 column, 1× HGR15 rail (two don't fit the 40 mm face), 2 blocks, T8×2 screw. |")
    w("")
    w("")
    w("## Control (owner's choice, 2026-11-20: CAN bus drivers)")
    w("")
    w("- **Drivers:** each joint motor has an MKS SERVO42D (NEMA17/14) or SERVO57D (NEMA23) closed-loop driver with a 14-bit encoder.")
    w("- **Bus:** one CAN bus from the ESP32, 4 wires through the arm: CAN H/L + 24 V/GND.")
    w("- **Firmware:** commands `JHOME`, `JMOVE`, `JPOS` (`docs/PROTOCOL.md`).")
    w("- **LeRobot plugin:** `arm_drive=\"stepper_can\"` sends the arm through the ESP32. Only the gripper stays on the Feetech bus.")
    w("- **Collision detection:** done by the drivers' locked-rotor (stall) protection, reported as a fault that names the joint.")
    w("- **Protocol source:** written from the official MKS CAN manual. The CC BY-NC library whickmott/MKSServoCAN was only used to cross-check the protocol facts.")
    w("")
    w("**Not checked yet:** self-collision across the joint ranges (the IK collision model is still the servo arm's), cable routing, the 24 V power budget, and the firmware build for the ESP32 (the PlatformIO registry is blocked here).")
    return "\n".join(L) + "\n"


def main():
    r = size()
    (HERE / "out").mkdir(exist_ok=True)
    (HERE / "out" / "sizing.json").write_text(json.dumps(r, indent=1, default=float))
    (ROOT / "docs" / "ARM_V2.md").write_text(report(r))
    for j, x in r["joints"].items():
        print(f"{j:14s} need {x['need']:6.2f}  usable {x['usable']:6.2f}  margin {x['margin']:.2f}  {x['speed_dps']:.0f} deg/s  sag {x['sag_mm']:.2f} mm")
    print("repeatability RSS", round(r["repeat_rss_mm"], 3), "mm; with 3 arcmin", round(r["repeat_rss_3arcmin_mm"], 3))
    print("mass", {k: round(v, 2) for k, v in r["mass"].items()})
    print("gantry", {k: round(v, 2) for k, v in r["gantry"].items()})


if __name__ == "__main__":
    main()
