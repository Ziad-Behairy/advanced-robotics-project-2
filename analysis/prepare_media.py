"""Copy the robot photos (resized JPEG) and extract stills from the demo video into report/figures/.

Videos are not in git (see .gitignore); if a video is missing the stills are kept as they are.
The side-by-side demo video carries its own overlay (mission time and state), so the stills are labeled by it.
"""
import cv2

from common import FIG, ROOT

PHOTOS = ["arm_side", "front_tof", "top_view"]
DEMO = ROOT / "video" / "A2_demo_side_by_side.mp4"
STILLS = {"searching": 3.0, "aligning": 5.5, "realign": 11.0, "chasing": 14.0, "arrived": 23.5}


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    out, lines = [], ["## Media"]
    for p in PHOTOS:
        img = cv2.imread(str(ROOT / "notes" / "photos" / f"{p}.png"))
        h, w = img.shape[:2]
        s = 900 / h
        img = cv2.resize(img, (int(w * s), 900), interpolation=cv2.INTER_AREA)
        dst = FIG / f"photo_{p}.jpg"
        cv2.imwrite(str(dst), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        out.append(dst)
    if DEMO.exists():
        cap = cv2.VideoCapture(str(DEMO))
        n, fps = cap.get(cv2.CAP_PROP_FRAME_COUNT), cap.get(cv2.CAP_PROP_FPS)
        lines.append(f"- demo video {DEMO.name}: {int(cap.get(3))}x{int(cap.get(4))}, {fps:.1f} fps, {n / fps:.1f} s")
        for name, t in STILLS.items():
            cap.set(cv2.CAP_PROP_POS_MSEC, t * 1000)
            ok, frame = cap.read()
            if ok:
                dst = FIG / f"still_{name}.jpg"
                cv2.imwrite(str(dst), frame, [cv2.IMWRITE_JPEG_QUALITY, 86])
                out.append(dst)
                lines.append(f"- still {name}: t = {t} s")
    else:
        lines.append("- demo video not found: stills not regenerated")
    return out, "\n".join(lines)


if __name__ == "__main__":
    print(main()[1])
