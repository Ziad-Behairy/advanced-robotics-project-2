#!/usr/bin/env python3
"""Render a replay video of one real run from its logs (no simulation: every value is recorded data).

Usage (venv):  python3 tools/make_replay.py RUN_DIR FRAMES_DIR OUT.mp4 [--fps 15]
- RUN_DIR: planner run folder (motor_voltage_full.csv with ros_time and odom columns)
- FRAMES_DIR: JPEG frames saved by tag_detector (save_frames_dir), named by ROS stamp in ns

Layout (1280x720): camera view + state panel on the left, top-view map + v(t), V1(t) on the right.
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import cv2
import imageio_ffmpeg
import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2' / 'rm_flatness_nav'))
from rm_flatness_nav.flatness import FlatTrajectory  # noqa: E402

STATE_COLORS = {'SEARCHING': '#d9822b', 'ALIGNING': '#2a5db0', 'CHASING': '#1b8a3a', 'ARRIVED': '#7d3c98'}
FRONT_OFFSET = 0.15   # m, chassis center -> front of the front wheels
DRIVE_SIGN = -1.0     # camera side = driver -x


def load_rows(run_dir):
    rows = list(csv.DictReader(open(Path(run_dir) / 'motor_voltage_full.csv')))
    out = []
    for r in rows:
        if not r.get('odom_x'):
            continue
        out.append({'t': float(r['t_s']), 'rt': float(r['ros_time']), 'state': r['state'],
                    'v': float(r['v_cmd']), 'w': float(r['w_cmd']), 'V': [float(r[f'V{i}']) for i in range(1, 5)],
                    'd': float(r['tag_d']) if r['tag_d'] else None,
                    'x': float(r['odom_x']), 'y': float(r['odom_y']), 'yaw': float(r['odom_yaw'])})
    return out


def heading(row):
    return row['yaw'] + (math.pi if DRIVE_SIGN < 0 else 0.0)   # camera direction in the odom frame


def plan_in_odom(rows):
    """Planned cubic path of the last chase, drawn in the odom frame, and the tag position estimate."""
    starts = [i for i in range(1, len(rows)) if rows[i]['state'] == 'CHASING' and rows[i - 1]['state'] != 'CHASING']
    if not starts:
        return None, None, None
    r0 = rows[starts[-1]]
    traj = FlatTrajectory(r0['d'])
    th = heading(r0)
    c, s = math.cos(th), math.sin(th)
    ts = np.linspace(0, traj.T, 100)
    xy = np.array([traj.sample(t)[:2] for t in ts])
    px = r0['x'] + c * xy[:, 0] - s * xy[:, 1]
    py = r0['y'] + s * xy[:, 0] + c * xy[:, 1]
    reach = r0['d'] + FRONT_OFFSET
    tag = (r0['x'] + c * reach, r0['y'] + s * reach)
    return (px, py), tag, r0['rt']


def robot_poly(row):
    th = heading(row)
    c, s = math.cos(th), math.sin(th)
    pts = np.array([[0.16, -0.12], [0.16, 0.12], [-0.16, 0.12], [-0.16, -0.12]])
    return np.column_stack([row['x'] + c * pts[:, 0] - s * pts[:, 1], row['y'] + s * pts[:, 0] + c * pts[:, 1]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('frames_dir')
    ap.add_argument('out')
    ap.add_argument('--fps', type=float, default=15.0)
    args = ap.parse_args()

    rows = load_rows(args.run_dir)
    frames = sorted(Path(args.frames_dir).glob('*.jpg'), key=lambda p: int(p.stem))
    fstamps = np.array([int(p.stem) * 1e-9 for p in frames])
    rtimes = np.array([r['rt'] for r in rows])
    plan, tag, _ = plan_in_odom(rows)
    t_start, t_end = rtimes[0], rtimes[-1]

    xs = [r['x'] for r in rows] + ([tag[0]] if tag else [])
    ys = [r['y'] for r in rows] + ([tag[1]] if tag else [])
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    half = max(max(xs) - min(xs), max(ys) - min(ys)) / 2 + 0.35

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    writer = imageio_ffmpeg.write_frames(args.out, (1280, 720), fps=args.fps, codec='libx264',
                                         output_params=['-pix_fmt', 'yuv420p', '-crf', '20'])
    writer.send(None)
    n = int((t_end - t_start) * args.fps) + 1
    for k in range(n):
        tau = t_start + k / args.fps
        i = max(0, int(np.searchsorted(rtimes, tau, side='right')) - 1)
        r = rows[i]
        fig.clf()
        # camera
        ax_cam = fig.add_axes([0.0, 0.42, 0.5, 0.58])
        j = int(np.searchsorted(fstamps, tau, side='right')) - 1
        if j >= 0:
            ax_cam.imshow(cv2.cvtColor(cv2.imread(str(frames[j])), cv2.COLOR_BGR2RGB))
        ax_cam.set_xticks([]); ax_cam.set_yticks([])
        ax_cam.set_title('What the robot camera sees (T3 = green box)', fontsize=12)
        # state panel
        ax_s = fig.add_axes([0.0, 0.0, 0.5, 0.42]); ax_s.axis('off')
        col = STATE_COLORS.get(r['state'], 'k')
        ax_s.text(0.5, 0.80, r['state'], ha='center', fontsize=34, weight='bold', color=col)
        d_txt = '--' if r['d'] is None else f"{r['d']:.2f} m"
        ax_s.text(0.5, 0.55, f"t = {r['t']:5.1f} s     v = {r['v']:.3f} m/s     ω = {r['w']:+.2f} rad/s     d = {d_txt}",
                  ha='center', fontsize=13)
        ax_s.text(0.5, 0.32, '   '.join(f'V{q + 1} = {r["V"][q]:+.2f} V' for q in range(4)), ha='center',
                  fontsize=12, family='monospace')
        ax_s.text(0.5, 0.08, 'Replay of recorded run data (camera frames, planner state, odometry)',
                  ha='center', fontsize=10, color='#666')
        # map
        ax_m = fig.add_axes([0.53, 0.40, 0.45, 0.57])
        past = rows[:i + 1]
        ax_m.plot([p['x'] for p in past], [p['y'] for p in past], color='#c0392b', lw=2.5, label='actual path (odometry)')
        if plan is not None and r['state'] in ('CHASING', 'ARRIVED'):
            ax_m.plot(plan[0], plan[1], '--', color='#1b8a3a', lw=2, label='planned cubic path z(t)')
        if tag is not None:
            ax_m.add_patch(Polygon([[tag[0] - .0775, tag[1] - .0775], [tag[0] + .0775, tag[1] - .0775],
                                    [tag[0] + .0775, tag[1] + .0775], [tag[0] - .0775, tag[1] + .0775]], color='k'))
            ax_m.text(tag[0], tag[1] + 0.12, 'T3', ha='center', fontsize=12, weight='bold', color='#1b8a3a')
        ax_m.add_patch(Polygon(robot_poly(r), closed=True, color='#9aa5b1', ec='k'))
        th = heading(r)
        ax_m.arrow(r['x'], r['y'], 0.22 * math.cos(th), 0.22 * math.sin(th), width=0.012, color='#c0392b')
        ax_m.set_xlim(cx - half, cx + half); ax_m.set_ylim(cy - half, cy + half); ax_m.set_aspect('equal')
        ax_m.set_title('Top view (odometry frame, m)', fontsize=12); ax_m.grid(alpha=0.3)
        ax_m.legend(loc='lower right', fontsize=9)
        # v(t) and V1(t)
        ax_p = fig.add_axes([0.58, 0.07, 0.38, 0.25])
        tt = [p['t'] for p in past]
        ax_p.plot(tt, [p['v'] for p in past], color='#2a5db0', lw=2)
        ax_p.set_xlim(0, rows[-1]['t']); ax_p.set_ylim(-0.01, 0.17)
        ax_p.set_xlabel('t (s)'); ax_p.set_ylabel('v (m/s)', color='#2a5db0')
        ax_v = ax_p.twinx()
        ax_v.plot(tt, [p['V'][0] for p in past], color='#c0392b', lw=1.5)
        vmin = min(min(p['V'][0] for p in rows), -0.2); vmax = max(max(p['V'][0] for p in rows), 1.2)
        ax_v.set_ylim(vmin - 0.1, vmax + 0.1); ax_v.set_ylabel('V1 (V)', color='#c0392b')
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        writer.send(np.ascontiguousarray(img))
    writer.close()
    print(f'wrote {args.out}: {n} frames, {n / args.fps:.1f} s, {len(frames)} camera frames used')


if __name__ == '__main__':
    main()
