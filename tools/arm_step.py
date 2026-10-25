#!/usr/bin/env python3
"""Move the RoboMaster arm to ONE absolute target and watch it, with a stall guard.

Usage (driver running):  python3 tools/arm_step.py X Z     # meters, e.g. 0.129 0.060
- Prints the arm position while it moves.
- If the target is not reached within --timeout s (e.g. blocked by the ToF sensor), it sends the
  current position as the new target so the arm holds still instead of pushing.
- Never sends a zero cmd_arm (the driver warns that can drop the arm).
"""
import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float)
    ap.add_argument('z', type=float)
    ap.add_argument('--timeout', type=float, default=4.0)
    ap.add_argument('--tol', type=float, default=0.006)
    args = ap.parse_args()
    if not (0.07 <= args.x <= 0.22 and 0.0 <= args.z <= 0.16):
        raise SystemExit(f'Target ({args.x}, {args.z}) outside the allowed box; refusing.')

    rclpy.init()
    node = rclpy.create_node('arm_step')
    pos = []
    node.create_subscription(PointStamped, 'arm_position', lambda m: pos.append((m.point.x, m.point.z)), 10)
    pub = node.create_publisher(Point, 'target_arm_position', 10)

    t0 = time.time()
    while not pos and time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not pos:
        raise SystemExit('No arm_position received: is the driver running?')
    print(f'start  x={pos[-1][0]:.3f} z={pos[-1][1]:.3f}  ->  target x={args.x:.3f} z={args.z:.3f}')

    pub.publish(Point(x=args.x, y=0.0, z=args.z))
    t0 = time.time()
    last_print = 0.0
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        x, z = pos[-1]
        err = np.hypot(x - args.x, z - args.z)
        elapsed = time.time() - t0
        if elapsed - last_print >= 0.5:
            print(f'  t={elapsed:4.1f}s  x={x:.3f} z={z:.3f}  err={err * 1000:.0f} mm')
            last_print = elapsed
        if err <= args.tol and elapsed > 1.0:
            print(f'REACHED x={x:.3f} z={z:.3f}')
            break
        if elapsed > args.timeout:
            pub.publish(Point(x=x, y=0.0, z=z))
            print(f'STALLED {err * 1000:.0f} mm from target: holding at x={x:.3f} z={z:.3f}')
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
