# AprilTag targets

## Printed tags
- Sheet: `docs/apriltags_tag36h11_T1_T2_T3_A4.pdf` (A4, one tag per page).
- Family `tag36h11`. T1 = ID 1, T2 = ID 2, T3 = ID 3 (mission target).
- Black square edge: 160 mm nominal. This is the `tag_size` passed to pupil-apriltags.
- Check after printing: the scale line must be 100 mm and the black square 160 mm.
  Measured: black edge = ___ mm, scale line = ___ mm (fill in with a ruler).

## What the detector gives
A black-and-white square marker with an ID. A library like pupil-apriltags finds the tag in the camera image.
Given the camera intrinsics (focal length, image center) and the printed tag size, it returns the tag pose
(position relative to the camera).

- Distance d = how far the tag is.
- Bearing θ = the left/right angle to the tag, θ = atan2(x_side, z_forward).
  Example: tag 0.3 m to the right and 1.5 m ahead gives θ = atan2(0.3, 1.5) = 0.197 rad ≈ 11.3°.
  That is above the 10° limit (0.18 rad), so the robot keeps aligning.

## Correction for the downward camera
- The camera is pitched down (arm at X = 105 mm, Y = 30 mm), so the camera z axis points into the floor,
  not along the floor. atan2(x_cam, z_cam) then mixes in the pitch and is not the ground bearing.
- Fix: rotate the tag position from the camera frame into the robot base frame (pitch from the arm pose,
  or from TF), then use ground-plane values:
  - d = sqrt(x_b² + y_b²) (horizontal distance, used for the 20 cm brake and for d0)
  - θ = atan2(y_b, x_b) (REP-103: +y left, so θ > 0 means the tag is to the left)
