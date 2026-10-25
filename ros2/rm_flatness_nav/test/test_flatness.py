import numpy as np
import pytest

from rm_flatness_nav.flatness import (
    V_MAX, FlatTrajectory, flat_inversion, poly_eval, solve_flat_polynomial, time_horizon)


def test_boundary_conditions_met():
    alpha = solve_flat_polynomial(0.0, 0.06, 1.4, 0.0, 14.0)
    z0, zd0, _ = poly_eval(alpha, 0.0)
    zT, zdT, _ = poly_eval(alpha, 14.0)
    assert np.allclose([z0, zd0, zT, zdT], [0.0, 0.06, 1.4, 0.0])


def test_time_horizon():
    assert time_horizon(1.8) == pytest.approx(16.0)
    assert time_horizon(1.5) == pytest.approx(13.0)
    assert time_horizon(0.4) == 4.0


@pytest.mark.parametrize('d0', [1.5, 1.6, 1.7, 1.8])
def test_peak_speed_below_vmax(d0):
    traj = FlatTrajectory(d0)
    speeds = [traj.sample(t)[2] for t in np.linspace(0.0, traj.T, 2001)]
    assert max(speeds) < V_MAX
    assert min(speeds) >= 0.0


def test_straight_plan_has_zero_turn_rate():
    traj = FlatTrajectory(1.6)
    assert all(abs(traj.sample(t)[3]) < 1e-12 for t in np.linspace(0.0, traj.T, 50))


def test_ends_at_standoff_and_stops():
    traj = FlatTrajectory(1.7)
    x, y, v, w = traj.sample(traj.T + 5.0)
    assert x == pytest.approx(1.5) and y == pytest.approx(0.0)
    assert v == 0.0 and w == 0.0


def test_inversion_on_circle():
    # x = R cos(wt), y = R sin(wt): v = R w, omega = w
    R, w, t = 0.5, 0.4, 1.3
    xd, yd = -R * w * np.sin(w * t), R * w * np.cos(w * t)
    xdd, ydd = -R * w**2 * np.cos(w * t), -R * w**2 * np.sin(w * t)
    v, omega = flat_inversion(xd, yd, xdd, ydd)
    assert v == pytest.approx(R * w) and omega == pytest.approx(w)


def test_inversion_at_rest_is_zero():
    assert flat_inversion(0.0, 0.0, 0.1, 0.1) == (0.0, 0.0)
