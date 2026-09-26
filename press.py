"""
E4: crank press (system): slider-crank + flywheel + motor, coupled through the torque curve.

The ram (slider) is driven by an in-line slider-crank for the stroke S; the press force F acts only over the
last s_w of the stroke, while the ram moves towards the bottom dead centre (x = a + b). Inertia and friction
are left out of the torque (reported).
    crank torque  T(theta) = F |dx/dtheta| in the working zone, else 0      (virtual work)
    work per stroke  W = F s_w = integral of T d(theta)                     (tested)
    mean torque  T_m = W / (2 pi);  energy fluctuation dE = max - min of the integral of (T - T_m) d(theta)
Flywheel on the fast shaft, as on real presses: it is the big belt pulley turning at motor rpm / 4 (a 4:1
V-belt, the rest of the reduction a gear stage to the crankshaft); the energy is the same on any shaft but the
inertia needed falls with the square of the speed (flywheel.py, press C_s 0.1 unless given).  The motor supplies only the
mean: P = T_m w / eta (eta = 0.85 for the belt / gear drive to the crankshaft);  without a flywheel it would
need T_max w / eta.  Crank and rod are sized for F (slider_crank.size, steel).  Units: mm, N, N*mm, rpm.
"""

import math

import flywheel
import slider_crank
import strength

ETA = 0.85
PINS = [6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 50]
CS_PRESS = 0.10
N = 3600


def torque_curve(a, b, F, s_w, n=N):
    """(theta list in rad, T list in N*mm) over one turn from the bottom dead centre (theta = 0, x = a + b)."""
    x_bdc = a + b
    ths, Ts = [], []
    for i in range(n):
        th = 2 * math.pi * i / n
        x = slider_crank.position(a, b, 0.0, th)[0]
        v = slider_crank.dx_dth(a, b, 0.0, th)
        working = x >= x_bdc - s_w and v > 0                  # moving towards the bottom dead centre
        ths.append(th)
        Ts.append(F * abs(v) if working else 0.0)
    return ths, Ts


BELT = 4.0


def design(F, S, s_w, spm, Cs=None, motor_rpm=1450.0, mat_key="steel", D_max=None):
    """Press for force F (N) over the last s_w (mm) of a stroke S at spm strokes per minute."""
    Cs = Cs or CS_PRESS
    g = slider_crank.synthesize(S, 0.0)
    if g is None:
        return None, [{"group": "Mechanism", "name": "Slider-crank", "util": 9.9, "detail": "no crank for this stroke"}]
    a, b = g["a"], g["b"]
    ths, Ts = torque_curve(a, b, F, min(s_w, S))
    h = 2 * math.pi / len(ths)
    W = sum(Ts) * h / 1000                                    # J
    T_mean = W / (2 * math.pi) * 1000                         # N*mm
    cum, s = [], 0.0
    for T in Ts:
        s += (T - T_mean) * h / 1000
        cum.append(s)
    dE = max(cum) - min(cum)
    fly_rpm = max(spm, motor_rpm / BELT)
    fw, fw_checks = flywheel.analyse(dE, fly_rpm, Cs, D_max)
    w = 2 * math.pi * spm / 60
    x_bdc = a + b
    rod = [F / math.cos(slider_crank.position(a, b, 0.0, t)[1]) for t, T in zip(ths, Ts) if T > 0]
    forc = {"F": F, "F_ab_max": max(rod) if rod else F, "M_crank_max": max(Ts), "M_rocker": 0.0}   # working zone only
    sec, s_checks = slider_crank.size(forc, a, b, strength.MATERIALS[mat_key], PINS)
    checks = list(fw_checks) + list(s_checks or [])
    info = {"a": a, "b": b, "S": S, "s_w": min(s_w, S), "F": F, "spm": spm, "W": W, "T_mean": T_mean,
            "T_max": max(Ts), "dE": dE, "Cs": Cs, "P_motor": T_mean / 1000 * w / ETA,
            "P_no_flywheel": max(Ts) / 1000 * w / ETA, "motor_rpm": motor_rpm, "drive_ratio": motor_rpm / spm,
            "flywheel_rpm": fly_rpm, "gear_ratio": fly_rpm / spm,
            "flywheel": fw, "section": sec, "curve": [(round(math.degrees(t), 1), round(T / 1000, 2))
                                                      for t, T in zip(ths[::30], Ts[::30])]}
    return info, checks
