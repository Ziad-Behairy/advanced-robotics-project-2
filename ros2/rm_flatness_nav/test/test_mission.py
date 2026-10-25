"""Closed-loop simulation of the mission state machine on a unicycle robot."""
import math

import pytest

from rm_flatness_nav.flatness import V_MAX
from rm_flatness_nav.mission import ALIGNING, ARRIVED, CHASING, SEARCHING, Mission

FOV = math.radians(45)     # half horizontal field of view (720p, fx ~ 629 px)
BLIND = 0.28               # m from the chassis front: closer floor points are below the image


def simulate(tag, heading0=0.0, tof_wall=None, dt=0.05, t_end=60.0):
    """tag = (x, y) of the tag center in the start frame; the robot front starts at the origin."""
    m = Mission()
    x = y = 0.0
    th = heading0
    states, speeds, t = [], [], 0.0
    while t < t_end:
        dx, dy = tag[0] - x, tag[1] - y
        fwd = dx * math.cos(th) + dy * math.sin(th)
        left = -dx * math.sin(th) + dy * math.cos(th)
        bearing = math.atan2(left, fwd)
        if fwd > BLIND and abs(bearing) < FOV:
            m.on_tag(t, math.hypot(fwd, left), bearing)
        if tof_wall is not None:
            m.on_tof(tof_wall - x)
        v, w = m.step(t, (x, y))
        states.append(m.state)
        speeds.append(v)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        t += dt
        if m.state == ARRIVED:
            break
    return m, (x, y), states, speeds


def first(states, s):
    return states.index(s) if s in states else None


@pytest.mark.parametrize('tag', [(1.6, 0.0), (1.8, 0.25), (1.5, -0.3)])
def test_reaches_standoff(tag):
    m, (x, y), states, speeds = simulate(tag)
    assert m.state == ARRIVED
    assert math.dist((x, y), tag) == pytest.approx(0.20, abs=0.04)
    assert max(speeds) < V_MAX
    assert first(states, ALIGNING) < first(states, CHASING)


def test_hidden_tag_search_then_align():
    # tag behind-left of the camera view: must sweep first (toward +, the default side)
    m, (x, y), states, _ = simulate((1.6, 0.0), heading0=math.radians(-90))
    assert states[0] == SEARCHING
    assert first(states, SEARCHING) < first(states, ALIGNING) < first(states, CHASING)
    assert m.state == ARRIVED
    assert math.dist((x, y), (1.6, 0.0)) == pytest.approx(0.20, abs=0.05)


def test_search_turns_toward_last_seen_side():
    m = Mission()
    m.on_tag(0.0, 1.5, -0.4)            # seen on the right
    _, w = m.step(1.0)                  # 1 s later: lost
    assert m.state == SEARCHING and w == pytest.approx(-0.30)


def test_aligning_turns_in_place():
    m = Mission()
    m.on_tag(0.0, 1.5, 0.5)
    v, w = m.step(0.0)
    assert m.state == ALIGNING and v == 0.0 and 0 < w <= 0.30


def test_tof_brake_overrides_plan():
    # a wall 0.5 m ahead, tag far beyond it: the ToF guard must stop the robot at 0.20 m from the wall
    m, (x, _), _, _ = simulate((1.8, 0.0), tof_wall=0.5)
    assert m.state == ARRIVED and 'ToF' in m.brake_reason
    assert 0.5 - x == pytest.approx(0.20, abs=0.02)


def test_arrived_holds_zero():
    m, _, _, _ = simulate((1.6, 0.0))
    assert m.step(1000.0, (5.0, 5.0)) == (0.0, 0.0)


def test_short_dropout_while_chasing_keeps_plan():
    m = Mission()
    for k in range(4):                      # centered detections -> CHASING
        m.on_tag(0.1 * k, 1.5, 0.0)
        m.step(0.1 * k, (0.0, 0.0))
    assert m.state == CHASING
    t0 = m.chase_t0
    v, _ = m.step(t0 + 1.0, (0.05, 0.0))   # 1 s without the tag: still chasing on the plan
    assert m.state == CHASING and v > 0
    m.step(t0 + 2.0, (0.10, 0.0))          # > 1.5 s without the tag, far from goal: search
    assert m.state == SEARCHING
