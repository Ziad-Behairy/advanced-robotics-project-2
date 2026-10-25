"""Shared paths, palette, figure style and loaders for the A2 analysis scripts.

Rules: scripts only READ data/ (and video/, notes/). They write vector PDF figures (plus JPEG stills and
photos) to report/figures/ and their numbers to analysis/results/summary.md.
Run with:  source ~/rm_setup.sh && python3 analysis/run_all.py
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
FIG = ROOT / "report" / "figures"
GEN = ROOT / "report" / "sections" / "generated"   # LaTeX snippets written by the scripts
RESULTS = ROOT / "analysis" / "results"
PKG = ROOT / "ros2" / "rm_flatness_nav"
sys.path.insert(0, str(PKG))   # import the package modules (flatness, voltage, geometry) without ROS

FINAL_RUN = DATA / "runs" / "20261026_015001"
FLOOR_RUNS = {   # run id -> standoff measured with tape by Ziad after the stop (from each notes.md)
    "20261026_011925": 0.21,
    "20261026_012232": 0.20,
    "20261026_015001": 0.20,
}
CALIB_CSV = DATA / "calibration" / "tag_range.csv"

# ---- palette --------------------------------------------------------------------------------
KFUPM = "#0b7a3e"
KFUPM_DARK = "#06542a"
INK = "#212529"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e4e6e3"
AXIS = "#c3c2b7"
STATE_COLOR = {"SEARCHING": "#eb6834", "ALIGNING": "#2a78d6", "CHASING": "#0b7a3e", "ARRIVED": "#7b3fa0"}
STATES = ["SEARCHING", "ALIGNING", "CHASING", "ARRIVED"]
WHEEL_COLOR = ["#0b7a3e", "#2a78d6", "#d4a017", "#c0392b"]          # V1 FR, V2 FL, V3 RL, V4 RR
WHEEL_NAME = ["V1 (FR)", "V2 (FL)", "V3 (RL)", "V4 (RR)"]
C_PLAN = "#0b7a3e"
C_MEAS = "#c0392b"
C_ACC = ["#0b7a3e", "#2a78d6", "#eb6834", "#7b3fa0", "#d4a017"]


def style():
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 8.5,
        "axes.titlesize": 9,
        "axes.titleweight": "bold",
        "axes.titlecolor": KFUPM_DARK,
        "axes.labelsize": 8.5,
        "axes.labelcolor": INK_2,
        "axes.edgecolor": AXIS,
        "axes.linewidth": 0.8,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "axes.axisbelow": True,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "xtick.labelcolor": INK_2,
        "ytick.labelcolor": INK_2,
        "xtick.major.size": 3,
        "ytick.major.size": 3,
        "legend.frameon": False,
        "legend.fontsize": 7.5,
        "lines.linewidth": 1.3,
        "figure.dpi": 150,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.03,
        "pdf.fonttype": 42,
    })


def save(fig, name):
    FIG.mkdir(parents=True, exist_ok=True)
    out = FIG / f"{name}.pdf"
    fig.savefig(out)
    plt.close(fig)
    return out


def load_csv(path):
    """CSV -> dict of arrays. Numeric columns become float arrays (empty cells NaN), others stay str."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    out = {}
    for k in rows[0]:
        col = [r[k] for r in rows]
        try:
            out[k] = np.array([float(v) if v != "" else np.nan for v in col])
        except ValueError:
            out[k] = np.array(col)
    return out


def load_raw_rows(path):
    with open(path, newline="") as f:
        return list(csv.reader(f))


def state_segments(t, states):
    """[(state, t_start, t_end)] contiguous segments; each tick covers [t - dt, t]."""
    segs = []
    dt = np.median(np.diff(t))
    start = t[0] - dt
    for i in range(1, len(t) + 1):
        if i == len(t) or states[i] != states[i - 1]:
            segs.append((states[i - 1], start, t[i - 1]))
            if i < len(t):
                start = t[i - 1]
    return segs


def shade_states(ax, t, states, alpha=0.13, labels=False):
    for s, a, b in state_segments(t, states):
        ax.axvspan(a, b, color=STATE_COLOR[s], alpha=alpha, lw=0)
        if labels and b - a > 1.2:
            ax.text((a + b) / 2, 1.0, s, transform=ax.get_xaxis_transform(), ha="center", va="bottom",
                    fontsize=6.5, color=STATE_COLOR[s], fontweight="bold")


def state_legend_handles():
    from matplotlib.patches import Patch
    return [Patch(color=STATE_COLOR[s], alpha=0.35, label=s) for s in STATES]
