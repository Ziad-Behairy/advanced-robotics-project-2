"""Wheel kinematics and motor armature voltage model (A2 Problem 3).

Wheel order follows the assignment: 1 = FR, 2 = FL, 3 = RL, 4 = RR (same as the SDK ESC order).
"""
import numpy as np

R_WHEEL = 0.05   # m
L_C = 0.20       # m, chassis radius
K_V = 0.38       # V s/rad, back-EMF constant
K_A = 0.06       # V s^2/rad, acceleration (inertia) term


def wheel_rates(vx, wz, r=R_WHEEL, lc=L_C):
    """[w1, w2, w3, w4] in rad/s from forward speed vx (m/s) and yaw rate wz (rad/s, CCW +) (eq. 4)."""
    w_fr = (vx + lc * wz) / r
    w_fl = (vx - lc * wz) / r
    return np.array([w_fr, w_fl, w_fl, w_fr])


def motor_voltages(w, w_prev, dt, kv=K_V, ka=K_A):
    """V_i = Kv w_i + Ka (w_i - w_i_prev) / dt (eq. 5)."""
    w = np.asarray(w, dtype=float)
    return kv * w + ka * (w - np.asarray(w_prev, dtype=float)) / dt


class VoltageModel:
    """Stateful version of eq. 5: keeps the previous wheel rates between control ticks."""

    def __init__(self):
        self.w_prev = np.zeros(4)

    def update(self, vx, wz, dt):
        w = wheel_rates(vx, wz)
        v = motor_voltages(w, self.w_prev, dt)
        self.w_prev = w
        return w, v
