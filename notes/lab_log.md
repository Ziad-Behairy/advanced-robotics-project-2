# Lab log, Assessment 2 (observations copied from the terminal output of the session, values verbatim)

## 2026-10-25
- 23:2x Tag print measured by Ziad (ruler): black edge 155 mm, 100 mm scale line = 98 mm.
- 23:55 Driver (AP, 720p) connected. Arm at rest, TF base_link <- camera_optical_link:
  translation [-0.073, -0.000, 0.272], RPY (deg) [-100.570, -0.000, -90.000] -> optical axis 10.6 deg below horizon.
- 23:55:24 First Wi-Fi drop: kernel "wlp4s0: deauthenticating from 34:d2:62:21:b8:cd by local choice (Reason: 3=DEAUTH_LEAVING)"; driver reconnected at 23:55:40.

## 2026-10-26
- 00:01 tag_detector with MoveArm(x=0.105, z=0.030) (A2 pose). T3 at ~0.5 m seen in 68/70 frames. TF afterwards:
  translation [-0.105, -0.000, 0.257], RPY (deg) [-71.918, -0.000, -90.000] (= 18 deg UP); tag in base_link
  (0.661, -0.033, +0.378) m, i.e. above the camera: TF inconsistent with a floor tag.
- 00:02-00:05 Arm kept moving with no active goal: arm_position (x, z) 0.136,0.077 -> 0.122,0.050 in 3 s, then
  0.138,0.087 -> 0.127,0.057. One absolute target_arm_position (0.135, 0.080) sent: arm settled at 0.136, 0.079 and held.
  Ziad: the gripper hits the front ToF sensor (photos notes/photos/). The sensor cannot be moved.
- 00:2x Arm resting on the ToF (x 0.129, z 0.041): camera from floor tag height 0.313 m, pitch 15.9 deg down;
  TF at the same time RPY (deg) [-67.699, 0, -90] (= 22 deg UP) -> arm TF not usable for the camera.
- After Ziad lifted the arm 2 cm by hand: camera from tag height 0.313 m, pitch 6.3 deg down; ToF /range_0 0.88 m.
- 00:19-00:34 AP mode: deauth "by local choice" at 00:19:14, 00:20:52, 00:21:58, 00:23:03, 00:24:37, 00:26:36,
  00:27:42, 00:28:47, 00:30:09, 00:31:15, 00:32:20, 00:33:50 (~65 s apart). Wi-Fi card: Realtek RTL8822CE, driver rtw_8822ce.
- 00:3x /etc/modprobe.d/rtw88-fix.conf (rtw88_pci disable_aspm=1, rtw88_core disable_lps_deep=1), both read Y after
  reload; drops continued (connected 00:37:57, deauth 00:38:57).
- 00:4x Router (STA) mode on home "Z&M" 2.4 GHz via QR code; robot 192.168.8.12. Check 00:51:10-00:54:10:
  wifi deauth events 0, driver disconnects 0, detector re-subscribes 0, T3 detections 10-15/s.
- 01:03 Range probe at 1.80 m tape: "NOT DETECTED in 8 s" while the detector processed ~14 frames/s (T3 seen 0).
- 01:10 Saved arm pose: arm_position x 0.1230, z 0.0670 (39 samples, std 0.0 mm).
- 01:1x Drive direction: 2 s of cmd_vel +0.05 m/s with wheels in the air; Ziad saw the top of the wheels move AWAY
  from the tag -> drive_sign = -1. Repeated wheels-off mission after the fix: Ziad confirmed wheels toward the tag.
- Floor runs, standoff by tape (Ziad): 011925 21 cm, 012232 20 cm, 015001 (final) 20 cm.
