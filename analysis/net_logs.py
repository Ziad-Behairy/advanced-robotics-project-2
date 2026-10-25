"""Engineering issue: Wi-Fi link drops, read from the raw driver logs in data/logs/."""
import re

import numpy as np

from common import DATA

LINE = re.compile(r"\[(\d+\.\d+)\] \[robomaster\]: (Connected|Disconnected|Try to connect via (\w+))")


def main():
    lines = ["## Driver connection logs"]
    for f in sorted((DATA / "logs").glob("driver_*.log")):
        mode, events = None, []
        for ln in f.read_text(errors="replace").splitlines():
            m = LINE.search(ln)
            if not m:
                continue
            if m.group(3):
                mode = mode or m.group(3)
            else:
                events.append((float(m.group(1)), m.group(2)))
        up = []
        for (t0, e0), (t1, e1) in zip(events, events[1:]):
            if e0 == "Connected" and e1 == "Disconnected":
                up.append(t1 - t0)
        n_drop = sum(e == "Disconnected" for _, e in events)
        span = events[-1][0] - events[0][0] if events else 0
        s = (f"- {f.name}: mode {mode}, {sum(e == 'Connected' for _, e in events)} connects, {n_drop} disconnects, "
             f"{span / 60:.1f} min from first to last event")
        if up:
            s += (f"; connected time before each drop: median {np.median(up):.1f} s, min {min(up):.1f} s, "
                  f"max {max(up):.1f} s, n = {len(up)}")
        lines.append(s)
    return [], "\n".join(lines)


if __name__ == "__main__":
    print(main()[1])
