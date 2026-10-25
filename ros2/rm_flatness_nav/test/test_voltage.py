import numpy as np
import pytest

from rm_flatness_nav.voltage import K_V, VoltageModel, motor_voltages, wheel_rates


def test_straight_drive_all_wheels_equal():
    assert np.allclose(wheel_rates(0.1, 0.0), [2.0] * 4)


def test_ccw_turn_right_side_faster():
    w = wheel_rates(0.0, 0.3)  # CCW in place: right wheels forward, left wheels back
    assert w[0] == pytest.approx(1.2) and w[3] == w[0]
    assert w[1] == pytest.approx(-1.2) and w[2] == w[1]


def test_steady_state_is_back_emf_only():
    w = wheel_rates(0.1, 0.0)
    assert np.allclose(motor_voltages(w, w, 0.1), K_V * 2.0)


def test_zero_at_rest():
    model = VoltageModel()
    model.update(0.1, 0.0, 0.1)
    model.update(0.0, 0.0, 0.1)          # braking tick: negative accel term
    _, v = model.update(0.0, 0.0, 0.1)   # stopped and held
    assert np.all(v == 0.0)


def test_acceleration_adds_voltage():
    model = VoltageModel()
    _, v = model.update(0.06, 0.0, 0.1)  # 0 -> 1.2 rad/s in 0.1 s
    assert np.allclose(v, 0.38 * 1.2 + 0.06 * 12.0)
