#!/usr/bin/env python3
"""Live mission view: robot camera (tag boxes) on the left, top-view map on the right.

Usage (venv, driver + tag_detector + planner):  python3 tools/live_view.py [--record OUT.mp4]
- Left: robot camera with tag boxes, state, distance, bearing and the velocity command.
- Right: top view from odometry: driven path (red), planned cubic path z(t) (green dashed, drawn when
  CHASING starts, from the measured d0) and T3 at the end of the plan.
- --record writes exactly this window's content to an MP4 (no screen capture). Press q to quit.
"""
import argparse
import math
import signal
import sys
from pathlib import Path

import cv2
import imageio_ffmpeg
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray, String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2' / 'rm_flatness_nav'))
from rm_flatness_nav.flatness import FlatTrajectory  # noqa: E402

CW, CH, MW = 960, 544, 480          # camera panel (h divisible by 16), map panel width
FPS = 15.0
DRIVE_SIGN = -1.0                   # camera side = driver -x
FRONT_OFFSET = 0.15                 # chassis center -> front of front wheels (m)
COLORS = {'SEARCHING': (43, 130, 217), 'ALIGNING': (176, 93, 42), 'CHASING': (58, 138, 27), 'ARRIVED': (152, 60, 125)}


class View:
    def __init__(self):
        self.img = None
        self.state = 'WAITING'
        self.d = self.b = None
        self.v = self.w = 0.0
        self.pose = None                  # (x, y, camera heading)
        self.path = []
        self.plan = None                  # (N x 2 array, tag xy)

    def on_state(self, s):
        if s == 'CHASING' and self.state != 'CHASING' and self.pose is not None and self.d is not None:
            x0, y0, th = self.pose
            traj = FlatTrajectory(self.d)
            pts = np.array([traj.sample(t)[:2] for t in np.linspace(0, traj.T, 80)])
            c, s_ = math.cos(th), math.sin(th)
            xy = np.column_stack([x0 + c * pts[:, 0] - s_ * pts[:, 1], y0 + s_ * pts[:, 0] + c * pts[:, 1]])
            reach = self.d + FRONT_OFFSET
            self.plan = (xy, (x0 + c * reach, y0 + s_ * reach))
        self.state = s

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        th = yaw + (math.pi if DRIVE_SIGN < 0 else 0.0)
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, th)
        if not self.path or math.dist(self.path[-1], self.pose[:2]) > 0.005:
            self.path.append(self.pose[:2])

    def camera_panel(self):
        canvas = np.zeros((CH, CW, 3), np.uint8)
        if self.img is not None:
            canvas[:] = cv2.resize(self.img, (CW, CH), interpolation=cv2.INTER_AREA)
        col = COLORS.get(self.state, (200, 200, 200))
        cv2.rectangle(canvas, (0, 0), (CW, 64), (25, 25, 25), -1)
        cv2.putText(canvas, self.state, (16, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.5, col, 4)
        d = '--' if self.d is None else f'{self.d:.2f} m'
        b = '--' if self.b is None else f'{np.degrees(self.b):+.1f} deg'
        cv2.putText(canvas, f'd = {d}   bearing = {b}', (370, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(canvas, f'cmd: v = {abs(self.v):.3f} m/s   w = {self.w:+.2f} rad/s', (370, 56),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(canvas, 'Robot camera (target T3 = green box)', (16, CH - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)
        return canvas

    def map_panel(self):
        panel = np.full((CH, MW, 3), 245, np.uint8)
        cv2.putText(panel, 'Top view (odometry)', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 40, 40), 2)
        if self.pose is None:
            return panel
        pts = list(self.path) + [self.pose[:2]]
        if self.plan is not None:
            pts += list(map(tuple, self.plan[0])) + [self.plan[1]]
        else:   # before the plan: show 1.8 m ahead of the camera so the map does not jump much later
            x, y, th = self.pose
            pts.append((x + 1.8 * math.cos(th), y + 1.8 * math.sin(th)))
        arr = np.array(pts)
        cx, cy = (arr.max(0) + arr.min(0)) / 2
        span = max(arr.max(0) - arr.min(0)) + 0.6
        scale = min(MW - 40, CH - 80) / span

        def px(p):   # world (x up/right) -> image
            return int(MW / 2 + (p[0] - cx) * scale), int(CH / 2 + 20 - (p[1] - cy) * scale)

        # grid every 0.5 m
        for g in np.arange(math.floor((cx - span) * 2) / 2, cx + span, 0.5):
            cv2.line(panel, px((g, cy - span)), px((g, cy + span)), (225, 225, 225), 1)
        for g in np.arange(math.floor((cy - span) * 2) / 2, cy + span, 0.5):
            cv2.line(panel, px((cx - span, g)), px((cx + span, g)), (225, 225, 225), 1)
        if self.plan is not None:
            xy, tag = self.plan
            for a, b in zip(xy[:-1:2], xy[1::2]):
                cv2.line(panel, px(a), px(b), (58, 138, 27), 3)
            t = px(tag)
            half = max(4, int(0.0775 * scale))
            cv2.rectangle(panel, (t[0] - half, t[1] - half), (t[0] + half, t[1] + half), (0, 0, 0), -1)
            cv2.putText(panel, 'T3', (t[0] - 14, t[1] - half - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (58, 138, 27), 2)
        if len(self.path) > 1:
            cv2.polylines(panel, [np.array([px(p) for p in self.path], np.int32)], False, (43, 57, 192), 3)
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        body = [(0.16, -0.12), (0.16, 0.12), (-0.16, 0.12), (-0.16, -0.12)]
        poly = np.array([px((x + c * u - s * v, y + s * u + c * v)) for u, v in body], np.int32)
        cv2.fillPoly(panel, [poly], (177, 165, 154))
        cv2.polylines(panel, [poly], True, (0, 0, 0), 2)
        cv2.arrowedLine(panel, px((x, y)), px((x + 0.25 * c, y + 0.25 * s)), (43, 57, 192), 3, tipLength=0.35)
        cv2.putText(panel, 'red: driven path   green: planned z(t)', (12, CH - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (60, 60, 60), 1)
        return panel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--record', default='')
    args = ap.parse_args()

    # stop cleanly on SIGINT/SIGTERM so the MP4 is finalized
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    node = rclpy.create_node('live_view')
    bridge = CvBridge()
    view = View()
    node.create_subscription(Image, 'tag/debug_image', lambda m: setattr(view, 'img', bridge.imgmsg_to_cv2(m, 'bgr8')), 1)
    node.create_subscription(String, 'mission/state', lambda m: view.on_state(m.data), 10)

    def on_rb(m):
        view.d, view.b = m.data[0], m.data[1]
    node.create_subscription(Float32MultiArray, 'tag/range_bearing', on_rb, 10)

    def on_cmd(m):
        view.v, view.w = m.linear.x, m.angular.z
    node.create_subscription(Twist, 'cmd_vel', on_cmd, 10)
    node.create_subscription(Odometry, 'odom', view.on_odom, 10)

    writer = None
    if args.record:
        writer = imageio_ffmpeg.write_frames(args.record, (CW + MW, CH), fps=FPS, codec='libx264',
                                             output_params=['-pix_fmt', 'yuv420p', '-crf', '20'])
        writer.send(None)
        print(f'recording window content to {args.record}', flush=True)

    name = 'RoboMaster live view'
    cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
    next_frame = 0.0
    clock = node.get_clock()
    try:
        while rclpy.ok() and not stop['now']:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = clock.now().nanoseconds * 1e-9
            if now < next_frame:
                continue
            next_frame = now + 1.0 / FPS if next_frame == 0.0 or now - next_frame > 0.5 else next_frame + 1.0 / FPS
            frame = np.hstack([view.camera_panel(), view.map_panel()])
            if writer is not None:
                writer.send(np.ascontiguousarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
            cv2.imshow(name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.close()
            print(f'saved {args.record}', flush=True)
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
