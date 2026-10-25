#!/usr/bin/env python3
"""Measure T3 distance from the chassis front for N seconds and print a summary line.

Uses only the camera: undoes the (unreliable) arm TF that tag_detector applied to recover the raw tag
position in the camera frame, then takes the floor distance with the camera height measured from the tag.
Usage (driver + tag_detector running):  python3 tools/tag_range_probe.py --tape 1.00 [--secs 5]
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2' / 'rm_flatness_nav'))
from rm_flatness_nav.geometry import quat_to_rot  # noqa: E402

CAM_BEHIND_FRONT = 0.149   # m, measured 2026-10-26 with the tag at 0.600 m (tape)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tape', type=float, required=True, help='tape distance chassis front -> tag center (m)')
    ap.add_argument('--secs', type=float, default=5.0)
    ap.add_argument('--height', type=float, default=0.313, help='camera height from the floor tag (m)')
    ap.add_argument('--csv', default='', help='append the raw result to this CSV file')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('tag_range_probe')
    buf = Buffer()
    TransformListener(buf, node)
    pts = []
    node.create_subscription(PointStamped, 'tag/point', lambda m: pts.append((m.point.x, m.point.y, m.point.z)), 10)
    t0 = time.time()
    while time.time() - t0 < args.secs:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not pts:
        print(f'tape {args.tape:.3f} m | NOT DETECTED in {args.secs:.0f} s')
        return
    tf = buf.lookup_transform('base_link', 'camera_optical_link', rclpy.time.Time())
    q, tr = tf.transform.rotation, tf.transform.translation
    rot, t = quat_to_rot(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z])
    p_cam = np.array([rot.T @ (np.array(p) - t) for p in pts])
    slant = np.linalg.norm(p_cam, axis=1)
    d_front = np.sqrt(np.maximum(slant**2 - args.height**2, 0.0)) - CAM_BEHIND_FRONT
    rate = len(pts) / args.secs
    if args.csv:
        path = Path(args.csv)
        new = not path.exists()
        with path.open('a') as f:
            if new:
                f.write('wall_time,tape_m,slant_mean_m,slant_std_m,d_front_mean_m,d_front_std_m,n_detections,secs,'
                        'height_m,cam_behind_front_m\n')
            f.write(f'{time.strftime("%Y-%m-%dT%H:%M:%S")},{args.tape:.3f},{slant.mean():.4f},{slant.std():.4f},'
                    f'{d_front.mean():.4f},{d_front.std():.4f},{len(pts)},{args.secs:.1f},{args.height:.3f},'
                    f'{CAM_BEHIND_FRONT:.3f}\n')
    print(f'tape {args.tape:.3f} m | camera {d_front.mean():.3f} m (std {d_front.std() * 1000:.1f} mm) | '
          f'error {(d_front.mean() - args.tape) * 100:+.1f} cm | {rate:.1f} detections/s')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
