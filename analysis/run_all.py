"""Regenerate every report figure, the generated LaTeX tables and analysis/results/summary.md.

    source ~/rm_setup.sh
    python3 analysis/run_all.py

Outputs: report/figures/*.pdf|jpg, report/sections/generated/*.tex, analysis/results/summary.md.
"""
import subprocess
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import net_logs  # noqa: E402
import plot_camera  # noqa: E402
import plot_flatness  # noqa: E402
import plot_run  # noqa: E402
import plot_voltage  # noqa: E402
import prepare_media  # noqa: E402
from common import PKG, RESULTS, ROOT  # noqa: E402


def unit_tests():
    r = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", "test"], cwd=PKG,
                       capture_output=True, text=True)
    last = [ln for ln in r.stdout.splitlines() if ln.strip()][-1]
    return f"## Unit tests (ros2/rm_flatness_nav/test)\n- pytest: {last}"


def mission_sim():
    """Closed-loop simulation from test/test_mission.py: final distance to the tag and peak speed."""
    sys.path.insert(0, str(PKG / "test"))
    import math
    from test_mission import simulate
    lines = ["## Closed-loop mission simulation (test_mission.simulate)"]
    for tag, h0 in (((1.6, 0.0), 0.0), ((1.8, 0.25), 0.0), ((1.5, -0.3), 0.0), ((1.6, 0.0), math.radians(-90))):
        m, (x, y), states, speeds = simulate(tag, heading0=h0)
        lines.append(f"- tag {tag}, start heading {math.degrees(h0):.0f} deg: final state {m.state}, distance to tag "
                     f"{math.dist((x, y), tag):.3f} m, peak v {max(speeds):.4f} m/s, brake: {m.brake_reason}")
    return "\n".join(lines)


def main():
    parts = [f"# Analysis summary\nGenerated {datetime.now():%Y-%m-%d %H:%M} by analysis/run_all.py. "
             "Do not edit by hand; rerun the script.\n"]
    figs = []
    for mod in (plot_voltage, plot_flatness, plot_run, plot_camera, net_logs, prepare_media):
        outs, text = mod.main()
        figs += [o.relative_to(ROOT) for o in outs]
        parts.append(text)
    parts.append(unit_tests())
    parts.append(mission_sim())
    parts.append("## Figures\n" + "\n".join(f"- `{f}`" for f in figs))
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "summary.md").write_text("\n\n".join(parts) + "\n")
    print("\n".join(str(f) for f in figs))
    print(RESULTS / "summary.md")


if __name__ == "__main__":
    main()
