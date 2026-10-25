"""Problem 1: camera geometry numbers, range calibration and an offline detection-range study.

The detection study renders the real T3 bitmap (ros2/rm_flatness_nav/test/data/tag36h11_03.png, 155 mm black
edge) lying on the floor, seen by the calibrated 720p pinhole model at the measured camera height and pitch,
and runs pupil-apriltags on it. It is a simulation (ideal image, no blur from motion, no compression), so it
gives an upper bound on the real range; the real limits come from the calibration CSV and the runs.
"""
import math

import cv2
import numpy as np
import matplotlib.pyplot as plt

from common import (C_ACC, C_MEAS, C_PLAN, CALIB_CSV, KFUPM_DARK, MUTED, PKG, STATE_COLOR, load_csv, save, style)
from rm_flatness_nav.geometry import camera_to_level

K720 = np.array([[628.58, 0, 638.02], [0, 625.79, 361.60], [0, 0, 1]])   # rm_camera_calibration_720
SIZE = 0.155          # m, measured black edge
HEIGHT = 0.313        # m, camera height from the floor tag (detector log, calibration session)
HEIGHT_CAL = 0.310    # m, camera height used to re-derive the calibration (coordinator, final arm pose)
FINAL_ROWS = [2, 3, 4, 5]   # CSV data rows 3-6 (01:00:44-01:07:34): final arm pose; rows 1-2 earlier pose
PITCH = 5.3           # deg down, config/robot.yaml
OFFSET = 0.162        # m, camera behind the chassis front (config range_offset = -0.162)
TAG = cv2.imread(str(PKG / "test" / "data" / "tag36h11_03.png"), cv2.IMREAD_GRAYSCALE)


def camera_pose(pitch_deg, height):
    p = np.radians(pitch_deg)
    r_level = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])           # optical -> base, camera level
    r_pitch = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    return r_pitch @ r_level, np.array([0.0, 0.0, height])


def render(K, wh, rot, trans, x, y, yaw):
    """Same renderer as test/test_tag_pipeline.py, with the image size as a parameter."""
    w, h = wh
    half = 0.5 * SIZE * 10 / 8
    c, s = np.cos(yaw), np.sin(yaw)
    corners = [(-half, half), (half, half), (half, -half), (-half, -half)]
    pts = np.array([[x + c * u - s * v, y + s * u + c * v, 0.0] for u, v in corners])
    pc = (rot.T @ (pts - trans).T).T
    if np.any(pc[:, 2] <= 0.05):
        return None
    uv = (K @ pc.T).T
    uv = (uv[:, :2] / uv[:, 2:]).astype(np.float32)
    big = cv2.resize(TAG, (1000, 1000), interpolation=cv2.INTER_NEAREST)
    hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [999, 0], [999, 999], [0, 999]]), uv)
    img = np.full((h, w), 170, np.uint8)
    warped = cv2.warpPerspective(big, hm, (w, h), flags=cv2.INTER_AREA)
    mask = cv2.warpPerspective(np.full_like(big, 255), hm, (w, h))
    img[mask > 0] = warped[mask > 0]
    return cv2.GaussianBlur(img, (3, 3), 0.8), uv


def detection_study():
    from pupil_apriltags import Detector
    det = Detector(families="tag36h11", quad_decimate=1.0, refine_edges=True)
    rot, trans = camera_pose(PITCH, HEIGHT)
    d_front = np.round(np.arange(0.10, 2.21, 0.05), 3)
    yaws = np.radians([0, 20, 40, 60, 80])
    res = {}
    for name, K, wh in (("720p", K720, (1280, 720)), ("360p", K720 * np.array([[.5], [.5], [1]]), (640, 360))):
        rate, err, px = [], [], []
        for df in d_front:
            hits, e, size = 0, [], []
            for yw in yaws:
                r = render(K, wh, rot, trans, df + OFFSET, 0.0, yw)
                if r is None:
                    continue
                img, uv = r
                if uv[:, 0].min() < 0 or uv[:, 1].max() > wh[1] or uv[:, 1].min() < 0:
                    continue     # part of the tag outside the image
                size.append(np.sqrt(abs(cv2.contourArea(uv))))
                params = (K[0, 0], K[1, 1], K[0, 2], K[1, 2])
                ds = det.detect(img, estimate_tag_pose=True, camera_params=params, tag_size=SIZE)
                ds = [x for x in ds if x.tag_id == 3]
                if ds:
                    hits += 1
                    p = camera_to_level(ds[0].pose_t, math.radians(PITCH))
                    e.append(p[0] - OFFSET - df)
            rate.append(hits / len(yaws))
            err.append(np.mean(e) if e else np.nan)
            px.append(np.mean(size) if size else np.nan)
        res[name] = (np.array(rate), np.array(err), np.array(px))
    return d_front, res


def main():
    style()
    lines = ["## Problem 1: camera geometry and detection"]
    fx, fy, cx, cy = K720[0, 0], K720[1, 1], K720[0, 2], K720[1, 2]
    hfov = math.degrees(math.atan(cx / fx)), math.degrees(math.atan((1280 - cx) / fx))
    vhalf = math.degrees(math.atan((720 - cy) / fy))
    low_edge = PITCH + vhalf
    blind_cam = HEIGHT / math.tan(math.radians(low_edge))
    lines += [f"- 720p intrinsics fx={fx} fy={fy} cx={cx} cy={cy}: horizontal half FOV {hfov[0]:.1f} / {hfov[1]:.1f} deg",
              f"- vertical half FOV below the axis {vhalf:.1f} deg; lower image edge {low_edge:.1f} deg below horizon",
              f"- lower edge meets the floor {blind_cam:.3f} m ahead of the camera = {blind_cam - OFFSET:.3f} m "
              f"from the chassis front (tag center); whole 155 mm tag visible only beyond about "
              f"{blind_cam - OFFSET + SIZE / 2:.3f} m"]
    # slant vs ground range at the 20 cm standoff
    g = 0.20 + OFFSET
    slant = math.hypot(g, HEIGHT)
    lines += [f"- standoff 0.20 m from the front: ground distance from camera {g:.3f} m, slant range {slant:.3f} m; "
              f"slant - offset = {slant - OFFSET:.3f} m (would brake {100 * (slant - OFFSET - 0.20):.1f} cm late)",
              f"- note example (camera 0.25 m up, 25 deg pitch, tag 0.20 m ahead): slant {math.hypot(0.20, 0.25):.3f} m"]

    # ---------- calibration ----------
    c = load_csv(CALIB_CSV)
    # horizontal camera distance from the logged slant range and the camera height
    h_cam = np.sqrt(c["slant_mean_m"] ** 2 - HEIGHT_CAL ** 2)
    final = np.isin(np.arange(len(h_cam)), FINAL_ROWS)
    off = c["tape_m"] - h_cam
    lines.append("- calibration rows (tape, horizontal camera distance, tape - camera, rate/s, final pose):")
    for i in range(len(h_cam)):
        lines.append(f"  - row {i+1}: {c['tape_m'][i]:.3f}, {h_cam[i]:.4f}, {off[i]:+.4f}, "
                     f"{c['n_detections'][i] / c['secs'][i]:.1f}, {bool(final[i])}")
    med = float(np.median(off[final]))
    A = np.c_[h_cam[final], np.ones(final.sum())]
    sc, of = np.linalg.lstsq(A, c["tape_m"][final], rcond=None)[0]
    resid = c["tape_m"][final] - (h_cam[final] - 0.162)
    lines.append(f"- height used for the calibration: {HEIGHT_CAL} m; final rows (1-based): {[i+1 for i in FINAL_ROWS]}")
    lines += [f"- final-pose rows: median(tape - camera) = {med:+.4f} m; lstsq tape = {sc:.4f} cam {of:+.4f}",
              f"- residual with the configured offset -0.162 m: " + ", ".join(f"{r*1000:+.1f} mm" for r in resid)]

    d_front, res = detection_study()
    for name, (rate, err, px) in res.items():
        ok = d_front[rate >= 0.999]
        anyd = d_front[rate > 0]
        lines.append(f"- render study {name}: all yaws detected from {ok.min():.2f} to {ok.max():.2f} m; "
                     f"some detected up to {anyd.max():.2f} m; tag size at 1.5 m "
                     f"{px[np.argmin(abs(d_front - 1.5))]:.1f} px; mean range error (detected) "
                     f"{np.nanmean(np.abs(err)) * 1000:.1f} mm, max {np.nanmax(np.abs(err)) * 1000:.1f} mm")

    fig, axs = plt.subplots(1, 2, figsize=(7.4, 2.75), gridspec_kw={"wspace": 0.3})
    ax = axs[0]
    xx = np.linspace(0.4, 1.7, 10)
    ax.plot(xx, xx, color=MUTED, lw=0.8, ls="--", label="ideal")
    ax.plot(c["tape_m"][~final], h_cam[~final] - 0.162, "o", mfc="white", mec=MUTED, ms=5,
            label="earlier arm pose (rows 1-2, not used)")
    ax.plot(c["tape_m"][final], h_cam[final] - 0.162, "o", color=C_PLAN, ms=6,
            label="final arm pose (rows 3-6), offset $-0.162$ m")
    for i in np.where(final)[0]:
        ax.annotate(f"{(h_cam[i] - 0.162 - c['tape_m'][i]) * 1000:+.0f} mm", (c["tape_m"][i], h_cam[i] - 0.162),
                    xytext=(6, -10), textcoords="offset points", fontsize=6.8, color=KFUPM_DARK)
    ax.set_xlabel("tape distance, chassis front to tag center (m)")
    ax.set_ylabel("camera estimate $d$ (m)")
    ax.set_title("range calibration (data/calibration)")
    ax.legend(fontsize=6.4, loc="upper left")

    ax = axs[1]
    for col, (name, (rate, err, px)) in zip((C_PLAN, C_ACC[1]), res.items()):
        ax.plot(d_front, rate * 100, color=col, lw=1.5, drawstyle="steps-mid", label=f"render study {name}")
    ax.set_xlabel("tag distance from chassis front (m)")
    ax.set_ylabel("rendered views detected (%)")
    ax.set_ylim(-5, 108)
    ax.axvspan(1.5, 1.8, color=STATE_COLOR["SEARCHING"], alpha=0.10, lw=0)
    ax.text(1.52, 92, "spec\nrange", fontsize=6.5, color=STATE_COLOR["SEARCHING"])
    axb = ax.twinx()
    rates = c["n_detections"] / c["secs"]
    axb.plot(c["tape_m"][~final], rates[~final], "o", mfc="white", mec=C_MEAS, ms=5)
    axb.plot(c["tape_m"][final], rates[final], "o", color=C_MEAS, ms=6)
    axb.plot([1.8], [0], "x", color=C_MEAS, ms=7, mew=1.6)
    axb.annotate("1.80 m: not detected", (1.8, 0), xytext=(1.25, 2.0), fontsize=6.5, color=C_MEAS,
                 arrowprops=dict(arrowstyle="-", color=C_MEAS, lw=0.6))
    axb.set_ylabel("real detections per second", color=C_MEAS)
    axb.set_ylim(-1, 18)
    axb.tick_params(axis="y", colors=C_MEAS, labelcolor=C_MEAS)
    axb.spines["right"].set_visible(True); axb.grid(False)
    ax.plot([], [], "o", color=C_MEAS, ms=5, label="real robot (calibration probe)")
    ax.legend(fontsize=6.4, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)
    ax.set_title("detection range")
    out = save(fig, "p1_range")
    return [out], "\n".join(lines)


if __name__ == "__main__":
    outs, txt = main()
    print(txt)
