"""Supervisory state machine for the A2 mission (Problem 1 ii, iii and Problem 2), free of ROS.

States:
- SEARCHING: tag not seen for more than 0.45 s. v = 0, rotate in place at 0.30 rad/s toward the side
  where the tag was last seen.
- ALIGNING: tag visible. v = 0, rotate in place until |bearing| <= 0.18 rad (10 deg).
- CHASING: follow the cubic flat-output plan built from the distance d0 measured when alignment finished.
  A small bearing feedback is added to the plan's omega (the plan itself is a straight line, omega = 0).
- ARRIVED: active brake, v = 0 and omega = 0, held for good.

Brake (ARRIVED) as soon as any of these holds:
- camera distance d <= 0.20 m (while the tag is visible),
- front ToF range <= 0.20 m,
- odometry shows the robot has covered the planned distance d0 - 0.20,
- the plan time T has elapsed (the plan itself ends at zero speed).
Near the end the floor tag leaves the camera view (camera blind zone), so losing the tag after the plan has
brought the robot within blind_zone of the standoff point does not abort the chase. Farther out, short
detection dropouts (the 155 mm floor tag gives ~4 detections/s at 1.5 m) are tolerated for up to
chase_lost_timeout while the plan continues open loop; the bearing correction is paused while the tag is
not visible.
"""
import math
from dataclasses import dataclass

from rm_flatness_nav.flatness import STANDOFF, FlatTrajectory

SEARCHING, ALIGNING, CHASING, ARRIVED = 'SEARCHING', 'ALIGNING', 'CHASING', 'ARRIVED'


@dataclass
class MissionParams:
    lost_timeout: float = 0.45      # s, tag considered hidden after this (SEARCHING / ALIGNING)
    chase_lost_timeout: float = 1.5  # s, dropouts tolerated while CHASING (~4 detections/s at 1.5 m)
    search_rate: float = 0.30       # rad/s
    align_tol: float = 0.18         # rad
    align_gain: float = 1.0         # rad/s per rad
    align_min_rate: float = 0.10    # rad/s, overcomes the chassis dead band
    align_settle: int = 3           # consecutive centered detections before CHASING
    chase_gain: float = 0.8         # rad/s per rad, bearing correction while chasing
    max_rate: float = 0.30          # rad/s, any in-place or correction turn
    standoff: float = STANDOFF      # m
    tof_brake: float = 0.20         # m
    tof_min_valid: float = 0.02     # m, ignore readings below this (no echo / noise)
    blind_zone: float = 0.25        # m, remaining planned distance where tag loss is expected


def clamp(x, lim):
    return max(-lim, min(lim, x))


class Mission:

    def __init__(self, params=None, search_dir=1.0):
        self.p = params or MissionParams()
        self.state = SEARCHING
        self.search_dir = 1.0 if search_dir >= 0 else -1.0
        self.last_tag_time = None
        self.bearing = 0.0
        self.distance = None
        self.tof = None
        self.centered_count = 0
        self.traj = None
        self.chase_t0 = None
        self.chase_xy0 = None
        self.d0 = None
        self.brake_reason = ''

    # inputs
    def on_tag(self, now, distance, bearing):
        self.last_tag_time = now
        self.distance = distance
        self.bearing = bearing
        if abs(bearing) > 1e-3:
            self.search_dir = math.copysign(1.0, bearing)  # remember where it was last seen

    def on_tof(self, rng):
        self.tof = rng

    def tag_visible(self, now):
        return self.last_tag_time is not None and now - self.last_tag_time <= self.p.lost_timeout

    # control
    def step(self, now, odom_xy=None):
        """Return (v, omega) for this tick; updates self.state. odom_xy: (x, y) from /odom, or None."""
        p = self.p
        visible = self.tag_visible(now)

        if self.state == ARRIVED:
            return 0.0, 0.0

        if self.state == CHASING:
            reason = self._brake_reason(now, visible, odom_xy)
            if reason:
                self.state, self.brake_reason = ARRIVED, reason
                return 0.0, 0.0
            t = now - self.chase_t0
            _, _, v_ref, w_ref = self.traj.sample(t)
            remaining = self.traj.x_goal - self._progress(t, odom_xy)
            lost_long = self.last_tag_time is None or now - self.last_tag_time > p.chase_lost_timeout
            if lost_long and remaining > p.blind_zone:
                self.state = SEARCHING          # really lost while still far: search again
                self.centered_count = 0
                return 0.0, 0.0
            w = w_ref + (clamp(p.chase_gain * self.bearing, p.max_rate) if visible else 0.0)
            return v_ref, clamp(w, p.max_rate)

        if not visible:
            self.state = SEARCHING
            self.centered_count = 0
            return 0.0, self.search_dir * p.search_rate

        # tag visible: ALIGNING (also the exit from SEARCHING)
        self.state = ALIGNING
        if abs(self.bearing) <= p.align_tol:
            self.centered_count += 1
            if self.centered_count >= p.align_settle:
                self._start_chase(now, odom_xy)
                return self.step(now, odom_xy)
            return 0.0, 0.0
        self.centered_count = 0
        w = p.align_gain * self.bearing
        w = math.copysign(max(abs(w), p.align_min_rate), w)
        return 0.0, clamp(w, p.max_rate)

    def _start_chase(self, now, odom_xy):
        self.d0 = self.distance
        self.traj = FlatTrajectory(self.d0)
        self.chase_t0 = now
        self.chase_xy0 = odom_xy
        self.state = CHASING

    def _progress(self, t, odom_xy):
        if odom_xy is not None and self.chase_xy0 is not None:
            return math.dist(odom_xy, self.chase_xy0)
        return self.traj.sample(t)[0]

    def _brake_reason(self, now, visible, odom_xy):
        p = self.p
        if visible and self.distance is not None and self.distance <= p.standoff:
            return f'camera d={self.distance:.3f} m'
        if self.tof is not None and p.tof_min_valid < self.tof <= p.tof_brake:
            return f'ToF {self.tof:.3f} m'
        if odom_xy is not None and self.chase_xy0 is not None:
            covered = math.dist(odom_xy, self.chase_xy0)
            if covered >= self.traj.x_goal:
                return f'odometry {covered:.3f} m covered'
        if now - self.chase_t0 >= self.traj.T:
            return f'plan finished (T={self.traj.T:.1f} s)'
        return ''
