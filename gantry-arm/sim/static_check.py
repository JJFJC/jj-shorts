"""Step 1 analysis: static joint torques (MuJoCo inverse dynamics), spring sizing,
servo hold sag (forward sim), gantry deflection / wobble -> docs/SIM_REPORT.md.

Run:  python sim/static_check.py           (single thread, ~20 s)
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
import model as M  # noqa: E402
import params as P  # noqa: E402

ARM_JOINTS = ["base_yaw", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
FULL_REACH = {"shoulder_lift": 90.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0, "base_yaw": 0.0}
g = 9.81


class Arm:
    def __init__(self, spring_k: float):
        self.k = spring_k
        self.m = mujoco.MjModel.from_xml_string(M.to_mjcf(spring_k))
        self.d = mujoco.MjData(self.m)
        self.jid = {n: self.m.joint(n).id for n in ARM_JOINTS + ["gantry_x", "gantry_z", "gripper_left"]}
        self.qadr = {n: self.m.jnt_qposadr[i] for n, i in self.jid.items()}
        self.dadr = {n: self.m.jnt_dofadr[i] for n, i in self.jid.items()}
        self.payload_body = self.m.body("payload").id
        self.payload_mass = self.m.body_mass[self.payload_body]

    def set_payload(self, on: bool):
        self.m.body_mass[self.payload_body] = self.payload_mass if on else 0.0

    def set_pose(self, pose: dict, z_mm: float = P.Z_TRAVEL):
        self.d.qpos[:] = 0
        self.d.qvel[:] = 0
        self.d.qacc[:] = 0
        for n, deg in pose.items():
            self.d.qpos[self.qadr[n]] = math.radians(deg)
        self.d.qpos[self.qadr["gantry_z"]] = z_mm / 1000

    def torques(self, pose: dict, qacc: dict | None = None) -> dict:
        """Joint torque (N·m) the actuators must supply to hold `pose` (+ optional accelerations)."""
        self.set_pose(pose)
        for n, a in (qacc or {}).items():
            self.d.qacc[self.dadr[n]] = a
        mujoco.mj_inverse(self.m, self.d)
        return {n: float(self.d.qfrc_inverse[self.dadr[n]]) for n in ARM_JOINTS}

    def tcp(self) -> np.ndarray:
        mujoco.mj_kinematics(self.m, self.d)
        return self.d.site("tcp").xpos.copy()


# --------------------------------------------------------------------------- sweeps

SWEEP = {
    "shoulder_lift": np.arange(-100, 101, 10),
    "elbow_flex": np.arange(-140, 141, 10),
    "wrist_flex": np.arange(-110, 111, 10),
}


def worst_case(arm: Arm) -> dict:
    """Max |torque| per joint over the pitch workspace, payload on and off."""
    worst = {n: (0.0, None) for n in ARM_JOINTS}
    for payload in (True, False):
        arm.set_payload(payload)
        for s in SWEEP["shoulder_lift"]:
            for e in SWEEP["elbow_flex"]:
                for w in SWEEP["wrist_flex"]:
                    pose = {"shoulder_lift": s, "elbow_flex": e, "wrist_flex": w, "wrist_roll": 0, "base_yaw": 0}
                    t = arm.torques(pose)
                    for n, v in t.items():
                        if abs(v) > abs(worst[n][0]):
                            worst[n] = (v, dict(pose, payload=payload))
    arm.set_payload(True)
    return worst


def size_spring() -> tuple[float, list]:
    """Pick k (N/mm) minimising the worst-case shoulder torque over the workspace."""
    rows = []
    best = (1e9, 0.0)
    for k in np.round(np.arange(0.0, 2.01, 0.1), 2):
        arm = Arm(float(k))
        w = 0.0
        for payload in (True, False):
            arm.set_payload(payload)
            for s in SWEEP["shoulder_lift"]:
                for e in SWEEP["elbow_flex"][::2]:
                    for wr in SWEEP["wrist_flex"][::2]:
                        t = arm.torques({"shoulder_lift": s, "elbow_flex": e, "wrist_flex": wr})
                        w = max(w, abs(t["shoulder_lift"]))
        rows.append((float(k), w))
        if w < best[0]:
            best = (w, float(k))
    return best[1], rows


# --------------------------------------------------------------------------- servo sag (forward sim)


def hold_sag(arm: Arm, pose: dict, seconds: float = 3.0, kp_scale: float = 1.0, feedforward: bool = False) -> dict:
    """Simulate the position servos holding `pose`; return steady-state error per joint (deg) and TCP drop (mm).

    kp_scale: multiply servo P gain (STS P_Coefficient 16 -> 32 is ~2x).
    feedforward: offset each goal by the predicted gravity sag tau/kp (software gravity compensation).
    """
    m, d = arm.m, arm.d
    kp0 = m.actuator_gainprm[:, 0].copy()
    bias0 = m.actuator_biasprm[:, 1].copy()
    m.actuator_gainprm[:, 0] = kp0 * kp_scale
    m.actuator_biasprm[:, 1] = bias0 * kp_scale
    tau = arm.torques(pose)
    arm.set_pose(pose)
    mujoco.mj_forward(m, d)
    tcp_ideal = d.site("tcp").xpos.copy()
    for i in range(m.nu):
        jid = m.actuator_trnid[i][0]
        d.ctrl[i] = d.qpos[m.jnt_qposadr[jid]]
        jn = m.joint(jid).name
        if feedforward and jn in tau:
            d.ctrl[i] += tau[jn] / m.actuator_gainprm[i, 0]
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)
    err = {n: math.degrees(d.qpos[arm.qadr[n]] - math.radians(pose.get(n, 0.0))) for n in ARM_JOINTS}
    sat = {}
    for i in range(m.nu):
        lo, hi = m.actuator_forcerange[i]
        sat[m.actuator(i).name] = abs(d.actuator_force[i]) / hi
    drop = float((tcp_ideal - d.site("tcp").xpos)[2] * 1000)
    m.actuator_gainprm[:, 0] = kp0
    m.actuator_biasprm[:, 1] = bias0
    return {"err_deg": err, "tcp_drop_mm": drop, "force_frac": sat}


# --------------------------------------------------------------------------- structure


def structure(arm: Arm) -> dict:
    """Hand calcs for the gantry structure, worst case: Z at top, arm at full reach, payload on."""
    A = P.ARM
    arm.set_payload(True)
    arm.set_pose(FULL_REACH)
    mujoco.mj_kinematics(arm.m, arm.d)
    mujoco.mj_comPos(arm.m, arm.d)
    zc = arm.m.body("z_carriage").id
    # mass hanging on the Z carriage (carriage + platform + arm + payload) and its CoM
    sub = [b for b in range(arm.m.nbody) if _is_descendant(arm.m, b, zc)]
    m_hang = sum(arm.m.body_mass[b] for b in sub)
    com = sum(arm.m.body_mass[b] * arm.d.xipos[b] for b in sub) / m_hang
    col_y = 0.0
    lever_y = com[1] - col_y  # m, CoM offset from column centre
    W = m_hang * g
    M_col = W * lever_y  # N·m about X
    tcp = arm.d.site("tcp").xpos.copy()
    tcp_lever = tcp[1] - col_y
    zc_pos = arm.d.xpos[zc].copy()
    h = zc_pos[2] - (P.X_PROFILE[0] + 10) / 1000  # height of Z carriage above column base, m
    E_al = P.AL_E * 1e6
    out = {"m_hang_kg": m_hang, "com_y_mm": lever_y * 1000, "M_col_Nm": M_col, "h_mm": h * 1000, "tcp_lever_mm": tcp_lever * 1000}

    # (a) column bending under the end moment, strong axis (40 mm along Y)
    I_s = P.I_2040["strong"] * 1e-12
    th_col = M_col * h / (E_al * I_s)
    out["col_rot_deg"] = math.degrees(th_col)
    out["col_tcp_mm"] = th_col * tcp_lever * 1000 + M_col * h**2 / (2 * E_al * I_s) * 0  # horizontal shift ignored for sag

    # (b) Z carriage rocking on its V-wheels
    pitch = P.Z_CARRIAGE["row_pitch"] / 1000
    n = P.Z_CARRIAGE["wheels_per_row"]
    out["zc_wheel_load_N"] = M_col / pitch / n + W / (2 * n)
    for tag, kw in zip(("lo", "hi"), P.WHEEL_K_N_PER_MM):
        k = kw * 1000
        delta = (M_col / pitch) / (n * k)
        th = 2 * delta / pitch
        out[f"zc_tcp_mm_{tag}"] = th * (tcp_lever - 0.0) * 1000

    # (c) X carriage rocking (moment about X) on the 2060: wheels in the side V-slots, ~70 mm apart across Y
    s_y = (P.X_PROFILE[1] + 10) / 1000
    n_side = P.X_CARRIAGE["wheels"] // 2
    m_total = m_hang + sum(arm.m.body_mass[b] for b in [arm.m.body("x_carriage").id])
    for tag, kw in zip(("lo", "hi"), P.WHEEL_K_N_PER_MM):
        k = kw * 1000
        delta = (M_col / s_y) / (n_side * k)
        th = 2 * delta / s_y
        out[f"xc_tcp_mm_{tag}"] = th * (tcp_lever + 0) * 1000 + th * 0
    out["xc_wheel_load_N"] = M_col / s_y / n_side + m_total * g / (2 * n_side)

    # (d) printed PETG platform: ribbed cantilever, length = offset
    pw = P.PLATFORM
    L = pw["offset"] / 1000
    I_pl = _ribbed_I(pw) * 1e-12
    E_pg = P.PETG_E_PRINTED * 1e6
    turret_id = arm.m.body("turret").id
    sub_arm = [b for b in range(arm.m.nbody) if _is_descendant(arm.m, b, turret_id)]
    m_arm = sum(arm.m.body_mass[b] for b in sub_arm)
    com_arm = sum(arm.m.body_mass[b] * arm.d.xipos[b] for b in sub_arm) / m_arm
    M_arm = m_arm * g * (com_arm[1] - arm.d.xpos[turret_id][1])
    th_pl = M_arm * L / (E_pg * I_pl) + m_arm * g * L**2 / (2 * E_pg * I_pl)
    d_pl = M_arm * L**2 / (2 * E_pg * I_pl) + m_arm * g * L**3 / (3 * E_pg * I_pl)
    reach_from_j1 = tcp[1] - arm.d.xpos[turret_id][1]
    out.update(
        m_arm_kg=m_arm, M_arm_Nm=M_arm, plat_I_mm4=I_pl * 1e12, plat_rot_deg=math.degrees(th_pl),
        plat_tip_mm=d_pl * 1000, plat_tcp_mm=(d_pl + th_pl * reach_from_j1) * 1000,
        plat_stress_MPa=(M_arm + m_arm * g * L) * _ribbed_c(pw) / 1000 / I_pl / 1e6,
    )
    # J1 bearing moment = same as M_arm: STS3215 output must not carry it
    out["j1_moment_Nm"] = M_arm

    # (e) printed arm links as hollow boxes under the full-reach load
    out["links"] = {}
    for name, L_mm, (bw, bh), t in (("upper_arm", A["upper"], (30, 40), 3.0), ("forearm", A["fore"], (28, 36), 3.0)):
        I_l = (bw * bh**3 - (bw - 2 * t) * (bh - 2 * t) ** 3) / 12 * 1e-12
        # load beyond the link end: use inverse-dynamics joint torque at the link root as the tip moment
        out["links"][name] = I_l
    tq = arm.torques(FULL_REACH)
    t_el = abs(tq["elbow_flex"])
    t_sh = abs(tq["shoulder_lift"])
    tcp_link = 0.0
    for name, L_mm, M_root, lever in (("upper_arm", A["upper"], t_sh, (A["fore"] + A["wrist"] + A["coupler"] + A["tool_tcp"])), ("forearm", A["fore"], t_el, (A["wrist"] + A["coupler"] + A["tool_tcp"]))):
        I_l = out["links"][name]
        Lm = L_mm / 1000
        # conservative: whole root moment applied uniformly along the link
        th = M_root * Lm / (E_pg * I_l)
        d = M_root * Lm**2 / (2 * E_pg * I_l)
        tcp_link += (d + th * lever / 1000) * 1000
    out["links_tcp_mm"] = tcp_link

    # (f) natural frequencies of the column + hanging mass
    I_w = P.I_2040["weak"] * 1e-12
    k_col_x = 3 * E_al * I_w / h**3
    k_col_y = 3 * E_al * I_s / h**3
    pitch_x = P.X_CARRIAGE["wheel_pitch"] / 1000
    m_eff = m_hang + 0.24 * (0.73 * P.Z_COLUMN_LEN / 1000)
    fr = {}
    for tag, kw in zip(("lo", "hi"), P.WHEEL_K_N_PER_MM):
        k = kw * 1000
        # carriage rocking about Y: wheels at +-pitch/2 on both rails sides
        k_rot_y = 2 * n_side * k * (pitch_x / 2) ** 2 * 2 / 2
        k_rot_x = 2 * n_side * k * (s_y / 2) ** 2
        kx = 1 / (1 / k_col_x + h**2 / k_rot_y)
        ky = 1 / (1 / k_col_y + h**2 / k_rot_x)
        fr[tag] = dict(
            fx=math.sqrt(kx / m_eff) / (2 * math.pi),
            fy=math.sqrt(ky / m_eff) / (2 * math.pi),
            x_accel_shift_mm=m_eff * P.X_ACCEL / 1000 / kx * 1000,
        )
    out["freq"] = fr
    out["k_col_x"] = k_col_x

    # (g) drive torques
    zs = P.Z_SCREW
    dm = zs["dia"] - zs["lead"] / 2
    l = zs["lead"]
    mu = zs["mu_nut"]
    sec = 1 / math.cos(math.radians(15))
    F_lift = m_hang * g + 10.0  # + 10 N wheel preload drag
    T_raise = F_lift * dm / 2000 * (l + math.pi * mu * dm * sec) / (math.pi * dm - mu * l * sec)
    T_lower = F_lift * dm / 2000 * (math.pi * mu * dm * sec - l) / (math.pi * dm + mu * l * sec)
    lead_angle = math.degrees(math.atan(l / (math.pi * dm)))
    fric_angle = math.degrees(math.atan(mu * sec))
    rpm_z = P.Z_SPEED / l * 60
    m_x = m_total
    F_x = m_x * P.X_ACCEL / 1000 + 8.0
    r_p = P.X_PULLEY_TEETH * 2 / (2 * math.pi) / 1000
    out["drives"] = dict(
        z_force_N=F_lift, z_T_raise=T_raise, z_T_lower=T_lower, lead_angle=lead_angle, fric_angle=fric_angle,
        z_rpm=rpm_z, x_mass=m_x, x_force=F_x, x_T=F_x * r_p, x_rpm=P.X_SPEED / (P.X_PULLEY_TEETH * 2) * 60,
    )
    return out


def _ribbed_I(pw) -> float:
    """Second moment (mm^4) of plate + downward ribs about its neutral axis."""
    parts = [(pw["width"], pw["plate"], pw["plate"] / 2)]  # (b, h, centroid y from top)
    for _ in range(pw["ribs"]):
        parts.append((pw["rib_t"], pw["rib_h"], pw["plate"] + pw["rib_h"] / 2))
    A = sum(b * h for b, h, _ in parts)
    yc = sum(b * h * y for b, h, y in parts) / A
    return sum(b * h**3 / 12 + b * h * (y - yc) ** 2 for b, h, y in parts)


def _ribbed_c(pw) -> float:
    parts = [(pw["width"], pw["plate"], pw["plate"] / 2)] + [(pw["rib_t"], pw["rib_h"], pw["plate"] + pw["rib_h"] / 2)] * pw["ribs"]
    A = sum(b * h for b, h, _ in parts)
    yc = sum(b * h * y for b, h, y in parts) / A
    return max(yc, pw["plate"] + pw["rib_h"] - yc)


def _is_descendant(m, b, root) -> bool:
    while b > 0:
        if b == root:
            return True
        b = m.body_parentid[b]
    return False


# --------------------------------------------------------------------------- report


def capacity(joint: str) -> tuple[float, float, int]:
    s, n = P.JOINT_SERVOS[joint]
    return s.stall_Nm * n, s.rated_Nm * n, n


def elbow_options(k: float) -> list:
    """What-if study for the flagged elbow: returns rows (label, J2 Nm, J3 Nm, J3 stall frac, J3 rated frac)."""
    cases = [
        ("Baseline: 1x STS3250, 500 g, 450 mm", {}, 1, 0.0),
        ("Payload target 400 g", {("MASS", "payload"): 400.0}, 1, 0.0),
        ("Payload target 300 g", {("MASS", "payload"): 300.0}, 1, 0.0),
        ("Forearm 160 -> 120 mm (reach 410 mm)", {("ARM", "fore"): 120.0}, 1, 0.0),
        ("2x STS3250 at elbow (ganged like J2)", {}, 2, P.STS3250.mass_g),
    ]
    rows = []
    for label, ch, n, extra_g in cases:
        saved_m, saved_a = dict(P.MASS), dict(P.ARM)
        for (table, key), v in ch.items():
            getattr(P, table)[key] = v
        arm = Arm(k)
        if extra_g:
            # second elbow servo sits at the elbow: add its weight to the upper-arm body at the elbow
            b = arm.m.body("upper_arm").id
            m0, c0 = arm.m.body_mass[b], arm.m.body_ipos[b].copy()
            m1 = extra_g / 1000
            arm.m.body_ipos[b] = (m0 * c0 + m1 * np.array([0, 0, P.ARM["upper"] / 1000])) / (m0 + m1)
            arm.m.body_mass[b] = m0 + m1
        arm.set_payload(True)
        t = arm.torques(FULL_REACH)
        P.MASS.clear(); P.MASS.update(saved_m)
        P.ARM.clear(); P.ARM.update(saved_a)
        j3 = abs(t["elbow_flex"])
        rows.append((label, abs(t["shoulder_lift"]), j3, j3 / (P.STS3250.stall_Nm * n), j3 / (P.STS3250.rated_Nm * n)))
    return rows


def run() -> dict:
    k_opt, k_rows = size_spring() if P.SPRING["k"] is None else (P.SPRING["k"], [])
    res = {"k_opt": k_opt, "k_rows": k_rows}
    for label, k in (("no_spring", 0.0), ("spring", k_opt)):
        arm = Arm(k)
        arm.set_payload(True)
        fr = arm.torques(FULL_REACH)
        # J1 is gravity-free (vertical axis): report the torque for a 3 rad/s^2 yaw acceleration at full reach
        fr_dyn = arm.torques(FULL_REACH, {"base_yaw": 3.0})
        fr["base_yaw"] = fr_dyn["base_yaw"]
        res[label] = {
            "full_reach": fr,
            "worst": worst_case(arm),
            "sag": hold_sag(arm, FULL_REACH),
            "sag_p2x": hold_sag(arm, FULL_REACH, kp_scale=2.0),
            "sag_ff": hold_sag(arm, FULL_REACH, feedforward=True),
        }
        arm.set_pose(FULL_REACH)
        res["reach_mm"] = float(arm.tcp()[1] - arm.d.xpos[arm.m.body("turret").id][1]) * 1000
    res["structure"] = structure(Arm(k_opt))
    res["elbow_options"] = elbow_options(k_opt)
    a, b = P.SPRING["a"], P.SPRING["b"]
    th = math.radians(max(abs(x) for x in P.JOINT_LIMITS["shoulder_lift"]))
    res["spring_Fmax_N"] = k_opt * math.sqrt(a * a + b * b - 2 * a * b * math.cos(th))
    res["spring_Fh_N"] = k_opt * math.sqrt(a * a + b * b)
    # gripper: rack & pinion, two friction fingers holding the payload against gravity
    Gp = P.GRIPPER
    F_grip = P.MASS["payload"] / 1000 * g * Gp["safety"] / (2 * Gp["mu_finger"])
    res["gripper_T"] = F_grip * Gp["pinion_r"] / 1000
    res["gripper_F"] = F_grip
    return res


def write_outputs(res: dict):
    (ROOT / "mjcf").mkdir(exist_ok=True)
    (ROOT / "urdf").mkdir(exist_ok=True)
    (ROOT / "mjcf" / "gantry_arm.xml").write_text(M.to_mjcf(res["k_opt"]))
    (ROOT / "urdf" / "gantry_arm.urdf").write_text(M.to_urdf())
    out = HERE / "out"
    out.mkdir(exist_ok=True)
    (out / "results.json").write_text(json.dumps(res, indent=1, default=lambda o: o.tolist()))


if __name__ == "__main__":
    r = run()
    write_outputs(r)
    print(json.dumps({k: r[k] for k in ("k_opt", "reach_mm", "gripper_T")}, indent=1))
    for lab in ("no_spring", "spring"):
        print(lab, {k: round(v, 3) for k, v in r[lab]["full_reach"].items()})
        print("  worst", {k: round(v[0], 3) for k, v in r[lab]["worst"].items()})
        for sk in ("sag", "sag_p2x", "sag_ff"):
            print("  ", sk, {k: round(v, 2) for k, v in r[lab][sk]["err_deg"].items()}, round(r[lab][sk]["tcp_drop_mm"], 1))
    s = r["structure"]
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in s.items() if k not in ("links",)})
