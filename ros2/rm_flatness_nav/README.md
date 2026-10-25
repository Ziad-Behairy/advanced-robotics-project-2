# rm_flatness_nav

ROS 2 Jazzy package for SCE 672 Assessment 2 on a DJI RoboMaster EP Core: AprilTag search and centering,
differential flatness pursuit to a 20 cm standoff, and motor armature voltage logging.

## Nodes
| Node | File | Role |
|---|---|---|
| `tag_detector` | `rm_flatness_nav/tag_detector.py` | pupil-apriltags on `camera/image_color`; publishes `tag/range_bearing` [d from chassis front, bearing, margin, stamp], `tag/point`, `tag/debug_image` |
| `flatness_planner` | `rm_flatness_nav/planner.py` | state machine SEARCHING / ALIGNING / CHASING / ARRIVED, cubic flat-output plan, brake, `cmd_vel`, V1..V4 log |

Core logic (ROS free, unit tested): `flatness.py` (eq. 1-3), `voltage.py` (eq. 4-5), `mission.py` (state machine),
`geometry.py` (frames, camera pose from the floor tag).

## Run
```bash
source ~/rm_setup.sh
# terminal 1: driver (router mode, 720p, front ToF)
ros2 launch robomaster_ros ep.launch conn_type:=sta video_resolution:=720 chassis_timeout:=0.6 tof_0:=true
# terminal 2: detector + planner with the calibrated parameters
ros2 launch rm_flatness_nav mission.launch.py
```
Each run writes `data/runs/<timestamp>/motor_voltage_log.csv` (1 s rows, printed live) and
`motor_voltage_full.csv` (20 Hz).

## Tests
```bash
cd ros2/rm_flatness_nav && python -m pytest -q test     # 36 tests, no robot needed
```

## Robot-specific parameters (`config/robot.yaml`)
- `tag_size 0.155` (measured print), `camera_pitch_deg 5.3`, `range_offset -0.162` (range calibration)
- `drive_sign -1`: the driver's +x points away from the camera/tag on this robot
- `arm_x_saved 0.123`, `arm_z_saved 0.067`: the planner only runs at the calibrated arm pose
