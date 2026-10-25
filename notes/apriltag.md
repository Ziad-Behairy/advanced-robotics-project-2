# AprilTag targets

## Printed tags
- Sheet: `docs/apriltags_tag36h11_T1_T2_T3_A4.pdf` (A4, one tag per page).
- Family `tag36h11`. T1 = ID 1, T2 = ID 2, T3 = ID 3 (mission target).
- Black square edge: 160 mm nominal. This is the `tag_size` passed to pupil-apriltags.
- Check after printing: the scale line must be 100 mm and the black square 160 mm.
  Measured by Ziad (2026-10-25, ruler): black edge = 155 mm, scale line = 98 mm (printer scaled ~98%).
  The detector uses tag_size = 0.155 m (the measured black edge, not the nominal 160 mm).

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
  not along the floor.
- Range is the big error: the camera's straight-line range |p_cam| includes the camera height. Example with
  the camera 0.25 m up and pitched 25 deg: a tag 0.20 m ahead on the floor reads 0.32 m, so braking on it
  would stop about 12 cm late.
- Bearing error is small: atan2(x_cam, z_cam) is within about 2 deg of the ground bearing in the same example.
- Detection range (offline render study, 155 mm tag, ideal image, camera 0.20 to 0.30 m up): 360p loses the
  tag beyond about 1.0 m; 720p detects reliably to about 1.2 m and only sometimes at 1.5 m. A 250 mm tag
  gains about 0.2 m; 2x upscaling does not help. Real range is measured in step 4 (camera check).
- Fix: rotate the tag position from the camera frame into the robot base frame (pitch from the arm pose,
  or from TF), then use ground-plane values:
  - d = sqrt(x_b² + y_b²) (horizontal distance, used for the 20 cm brake and for d0)
  - θ = atan2(y_b, x_b) (REP-103: +y left, so θ > 0 means the tag is to the left)
