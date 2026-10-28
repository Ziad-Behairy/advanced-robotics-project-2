#!/usr/bin/env python3
"""Reference vs executed motion and tracking errors for the final run (chase that reached T3).

Reads data/runs/20261026_015001/motor_voltage_full.csv (20 Hz planner log with odometry) and writes:
- presentation/figures/*.png (slides) and report/figures/p4_tracking_*.pdf (report)
- analysis/results/tracking_error.md (every number quoted in the slides)

Reference: x_ref(t), v_ref(t) of the cubic flat-output plan built from d0 at the chase start (flatness.py).
Executed: distance covered since the chase start from /odom; speed = central difference of odometry over
a 0.5 s window (odometry arrives at ~10 Hz in Wi-Fi bursts, so a per-sample derivative shows fake spikes).
"""
import csv
import math
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2' / 'rm_flatness_nav'))
from rm_flatness_nav.flatness import FlatTrajectory  # noqa: E402

RUN = ROOT / 'data' / 'runs' / '20261026_015001' / 'motor_voltage_full.csv'
PNG = ROOT / 'presentation' / 'figures'
PDF = ROOT / 'report' / 'figures'
REF, ACT, SHIFT, ERR, LIM = '#0b7a3e', '#c0392b', '#7d3c98', '#2a5db0', '#888888'
WINDOW = 0.5   # s, velocity difference window


def load_chase():
    rows = list(csv.DictReader(open(RUN)))
    st = [r['state'] for r in rows]
    i0 = [i for i in range(1, len(rows)) if st[i] == 'CHASING' and st[i - 1] != 'CHASING'][-1]
    i1 = next(i for i in range(i0, len(rows)) if st[i] == 'ARRIVED')
    seg = rows[i0:i1 + 1]
    t = np.array([float(r['ros_time']) for r in seg])
    t -= t[0]
    x0, y0 = float(seg[0]['odom_x']), float(seg[0]['odom_y'])
    x_act = np.array([math.hypot(float(r['odom_x']) - x0, float(r['odom_y']) - y0) for r in seg])
    v_cmd = np.array([float(r['v_cmd']) for r in seg])
    return t, x_act, v_cmd, float(seg[0]['tag_d']), float(seg[0]['t_s'])


def main():
    t, x_act, v_cmd, d0, t_mission = load_chase()
    traj = FlatTrajectory(d0)
    x_ref = np.array([traj.sample(s)[0] for s in t])
    v_ref = np.array([traj.sample(s)[2] for s in t])

    new = np.r_[True, np.diff(x_act) != 0]          # samples where a new odometry message arrived
    tu, xu = t[new], x_act[new]
    v_act = np.array([(np.interp(s + WINDOW / 2, tu, xu) - np.interp(s - WINDOW / 2, tu, xu)) / WINDOW for s in t])
    inner = (t > WINDOW / 2) & (t < t[-1] - WINDOW / 2)

    e_x = x_act - x_ref
    e_v = v_act - v_ref
    taus = np.arange(0.0, 1.0, 0.01)
    rms_tau = [np.sqrt(np.mean((x_act - np.array([traj.sample(max(s - tau, 0.0))[0] for s in t])) ** 2)) for tau in taus]
    k = int(np.argmin(rms_tau))
    tau = taus[k]
    x_shift = np.array([traj.sample(max(s - tau, 0.0))[0] for s in t])

    stats = {
        'd0': d0, 'T': traj.T, 'x_goal': traj.x_goal, 'chase_start_mission_s': t_mission,
        'alpha_x': traj.alpha_x, 'samples': len(t), 'odom_updates': int(new.sum()),
        'ex_mean': e_x.mean(), 'ex_rms': np.sqrt(np.mean(e_x ** 2)), 'ex_max': np.abs(e_x).max(),
        'ex_max_t': t[np.argmax(np.abs(e_x))], 'ex_final': e_x[-1], 'x_final': x_act[-1],
        'ev_mean': e_v[inner].mean(), 'ev_rms': np.sqrt(np.mean(e_v[inner] ** 2)), 'ev_max': np.abs(e_v[inner]).max(),
        'vref_peak': v_ref.max(), 'vref_peak_t': t[np.argmax(v_ref)], 'vact_peak': v_act[inner].max(),
        'vcmd_minus_vref_max': np.abs(v_cmd - v_ref).max(), 'tau': tau, 'rms_tau': rms_tau[k], 'rms_0': rms_tau[0],
    }

    plt.rcParams.update({'font.size': 12, 'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.grid': True, 'grid.alpha': 0.3})
    PNG.mkdir(parents=True, exist_ok=True)

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(PNG / f'{name}.png', dpi=200)
        fig.savefig(PDF / f'p4_{name}.pdf')
        plt.close(fig)

    # 1. reference vs executed position
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.plot(t, x_ref, color=REF, lw=3, label='reference $x_{ref}(t)$ (cubic plan)')
    ax.plot(t, x_act, color=ACT, lw=2, label='executed (odometry)')
    ax.plot(t, x_shift, '--', color=SHIFT, lw=1.6, label=f'reference delayed by {tau:.2f} s')
    ax.axhline(traj.x_goal, color=LIM, ls=':', lw=1.5)
    ax.text(0.2, traj.x_goal + 0.02, f'goal $d_0-0.20$ = {traj.x_goal:.3f} m', color=LIM)
    ax.set_xlabel('time since chase start (s)')
    ax.set_ylabel('distance along path x (m)')
    ax.set_title(f'Position: plan vs robot  (d0 = {d0:.3f} m, T = {traj.T:.2f} s)')
    ax.legend(loc='lower right')
    save(fig, 'tracking_x')

    # 2. reference vs executed speed
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.plot(t, v_ref, color=REF, lw=3, label='reference $v_{ref}(t)=\\dot x_{ref}$')
    ax.plot(t, v_cmd, ':', color='k', lw=1.5, label='commanded (cmd_vel)')
    ax.plot(t[inner], v_act[inner], color=ACT, lw=2, label=f'executed (odometry, {WINDOW:.1f} s difference)')
    ax.axhline(0.16, color=LIM, ls='--', lw=1.5)
    ax.text(0.2, 0.162, '$v_{max}$ = 0.16 m/s', color=LIM)
    ax.annotate(f'peak {v_ref.max():.3f} m/s at {stats["vref_peak_t"]:.1f} s', (stats['vref_peak_t'], v_ref.max()),
                (stats['vref_peak_t'] + 1.2, 0.07), arrowprops=dict(arrowstyle='->', color=REF), color=REF)
    ax.annotate('starts at $\\dot x(0)$ = 0.06 m/s', (0, 0.06), (1.0, 0.02), arrowprops=dict(arrowstyle='->'))
    ax.set_xlabel('time since chase start (s)')
    ax.set_ylabel('forward speed v (m/s)')
    ax.set_ylim(-0.005, 0.19)
    ax.set_title('Speed: plan vs robot')
    ax.legend(loc='upper right', fontsize=10)
    save(fig, 'tracking_v')

    # 3. errors
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5.6), sharex=True)
    a1.plot(t, e_x * 100, color=ERR, lw=2, label='$e_x = x_{robot} - x_{ref}$')
    a1.plot(t, (x_act - x_shift) * 100, color=SHIFT, lw=1.4, ls='--', label=f'vs reference delayed {tau:.2f} s')
    a1.axhline(0, color='k', lw=0.8)
    a1.set_ylabel('$e_x$ (cm)')
    a1.legend(loc='lower right', fontsize=10)
    a1.set_title(f'Position error: RMS {stats["ex_rms"] * 100:.1f} cm, max {stats["ex_max"] * 100:.1f} cm, '
                 f'final {stats["ex_final"] * 100:+.1f} cm  (delayed ref: RMS {stats["rms_tau"] * 100:.2f} cm)',
                 fontsize=11)
    a2.plot(t[inner], e_v[inner] * 100, color=ERR, lw=2)
    a2.axhline(0, color='k', lw=0.8)
    a2.set_ylabel('$e_v$ (cm/s)')
    a2.set_xlabel('time since chase start (s)')
    a2.set_title(f'Speed error: RMS {stats["ev_rms"] * 100:.1f} cm/s, max {stats["ev_max"] * 100:.1f} cm/s', fontsize=11)
    save(fig, 'tracking_errors')

    res = ROOT / 'analysis' / 'results'
    res.mkdir(parents=True, exist_ok=True)
    a = stats['alpha_x']
    lines = [
        '# Tracking error, final run 20261026_015001 (generated by analysis/tracking_error.py)', '',
        f'- Chase that reached T3 starts at mission t = {stats["chase_start_mission_s"]:.2f} s; d0 = {d0:.3f} m, '
        f'T = {traj.T:.2f} s, goal x(T) = {traj.x_goal:.3f} m',
        f'- alpha_x = [{a[0]:.4f}, {a[1]:.4f}, {a[2]:.6f}, {a[3]:.7f}]',
        f'- {stats["samples"]} planner ticks (20 Hz), {stats["odom_updates"]} odometry updates',
        f'- Position error e_x: mean {stats["ex_mean"] * 100:+.2f} cm, RMS {stats["ex_rms"] * 100:.2f} cm, '
        f'max |e| {stats["ex_max"] * 100:.2f} cm at t = {stats["ex_max_t"]:.1f} s, final {stats["ex_final"] * 100:+.2f} cm '
        f'(covered {stats["x_final"]:.4f} m of {traj.x_goal:.4f} m)',
        f'- Best-fit delay between reference and robot: {tau:.2f} s; x RMS error {stats["rms_0"] * 100:.2f} cm -> '
        f'{stats["rms_tau"] * 100:.2f} cm with the delayed reference',
        f'- Speed error e_v ({WINDOW} s difference): mean {stats["ev_mean"] * 100:+.2f} cm/s, '
        f'RMS {stats["ev_rms"] * 100:.2f} cm/s, max |e| {stats["ev_max"] * 100:.2f} cm/s',
        f'- v_ref peak {stats["vref_peak"]:.4f} m/s at {stats["vref_peak_t"]:.1f} s; executed peak {stats["vact_peak"]:.3f} m/s',
        f'- |v_cmd - v_ref| max {stats["vcmd_minus_vref_max"]:.5f} m/s (planner sends the plan speed)',
        '- Tape-measured final standoff (Ziad): 20 cm (target 20 cm)',
    ]
    (res / 'tracking_error.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
