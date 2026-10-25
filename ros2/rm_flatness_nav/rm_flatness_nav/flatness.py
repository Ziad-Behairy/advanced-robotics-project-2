"""Differential flatness trajectory generation (A2 Problem 2).

Flat output z(t) = [x(t), y(t)]^T, each component a cubic in t over horizon T.
v and omega are recovered algebraically from the derivatives of z (no optimizer).
"""
import numpy as np

V_MAX = 0.16         # m/s, cruise speed limit (eq. 3)
STANDOFF = 0.20      # m, stop distance from the tag
T_MIN = 4.0          # s, shortest allowed horizon (eq. 3)
XDOT0 = 0.06         # m/s, initial forward speed boundary condition


def solve_flat_polynomial(z0, zdot0, zf, zdotf, T):
    """Cubic coefficients [a0, a1, a2, a3] that meet position and velocity at t = 0 and t = T (eq. 2)."""
    A = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [1.0, T, T**2, T**3],
        [0.0, 1.0, 2.0 * T, 3.0 * T**2],
    ])
    b = np.array([z0, zdot0, zf, zdotf], dtype=float)
    return np.linalg.solve(A, b)


def time_horizon(d0, v_max=V_MAX, standoff=STANDOFF, t_min=T_MIN):
    """T = max(4, 1.6 (d0 - 0.20) / v_max) (eq. 3). The 1.6 factor keeps the cubic's peak speed below v_max."""
    return max(t_min, 1.6 * (d0 - standoff) / v_max)


def poly_eval(alpha, t):
    """Position, velocity and acceleration of a cubic at time t."""
    a0, a1, a2, a3 = alpha
    z = a0 + a1 * t + a2 * t**2 + a3 * t**3
    zd = a1 + 2.0 * a2 * t + 3.0 * a3 * t**2
    zdd = 2.0 * a2 + 6.0 * a3 * t
    return z, zd, zdd


def flat_inversion(xd, yd, xdd, ydd, eps=1e-6):
    """v = sqrt(xd^2 + yd^2), omega = (xd ydd - yd xdd) / (xd^2 + yd^2) (eq. 1).

    omega is undefined at zero speed (heading is not observable from z), so it is set to 0 there.
    """
    speed_sq = xd**2 + yd**2
    v = np.sqrt(speed_sq)
    omega = (xd * ydd - yd * xdd) / speed_sq if speed_sq > eps else 0.0
    return v, omega


class FlatTrajectory:
    """Straight pursuit plan from the robot (origin, facing the tag) to the standoff point.

    Built once when CHASING starts, in the robot frame at that moment.
    """

    def __init__(self, d0, y_tag=0.0, xdot0=XDOT0):
        self.d0 = d0
        self.T = time_horizon(d0)
        self.x_goal = d0 - STANDOFF
        self.alpha_x = solve_flat_polynomial(0.0, xdot0, self.x_goal, 0.0, self.T)
        self.alpha_y = solve_flat_polynomial(0.0, 0.0, y_tag, 0.0, self.T)

    def sample(self, t):
        """Reference (x, y, v, omega) at time t. Holds the end state after T."""
        t = min(max(t, 0.0), self.T)
        x, xd, xdd = poly_eval(self.alpha_x, t)
        y, yd, ydd = poly_eval(self.alpha_y, t)
        if t >= self.T:
            return x, y, 0.0, 0.0
        v, omega = flat_inversion(xd, yd, xdd, ydd)
        return x, y, v, omega
