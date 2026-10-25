# Analysis summary
Generated 2026-10-26 02:30 by analysis/run_all.py. Do not edit by hand; rerun the script.


## Problem 3: voltages (final run 20261026_015001)
- Constants: r = 0.05 m, Lc = 0.2 m, Kv = 0.38, Ka = 0.06, dt = 0.05 s (20 Hz control)
- motor_voltage_log.csv rows: 26 (t = 0 .. 25 s)
- motor_voltage_full.csv ticks: 506, t = 0.050 .. 25.300 s
- Tick interval from ros_time: mean 50.0 ms, std 0.3 ms, min 47.0 ms, max 53.0 ms
- State segments: SEARCHING -0.00-5.35 s; ALIGNING 5.35-6.20 s; SEARCHING 6.20-6.25 s; ALIGNING 6.25-6.70 s; SEARCHING 6.70-8.45 s; ALIGNING 8.45-8.55 s; CHASING 8.55-10.45 s; SEARCHING 10.45-10.60 s; ALIGNING 10.60-11.55 s; CHASING 11.55-22.25 s; ARRIVED 22.25-25.30 s
- Peak commanded v: 0.1371 m/s at t = 16.05 s
- V1 range: -2.505 .. 1.896 V; all wheels range -2.505 .. 2.694 V
- ARRIVED ticks: 61, max |V| while ARRIVED: 0.041 V (first ARRIVED tick t = 22.300 s)
- Steady search turn: w = 0.30 rad/s -> |w_i| = 1.200 rad/s, |V| = 0.456 V; start step adds Ka*1.2/0.05 = 1.440 V -> 1.896 V
- Ticks with |V1| > 1.3 V (step changes): t=0.05 -->SEARCHING V1=+1.896 V2=-1.896, t=8.50 SEARCHING->ALIGNING V1=-1.440 V2=+1.440, t=8.60 ALIGNING->CHASING V1=+1.650 V2=+2.142, t=10.50 CHASING->SEARCHING V1=-2.505 V2=-2.505, t=10.55 SEARCHING->SEARCHING V1=-1.896 V2=+1.896
- CHASING ticks with dw1/dt < 0: 118, Ka*dw1/dt min -0.080 V
- CHASING ticks with any negative V_i: 3

## Problem 2: flatness plans
- d0=1.440 (final run, 1st chase (cut at 10.45 s)): T=12.4000 s, alpha_x=[ 0.        0.06      0.014516 -0.000911], x(T)=1.2400 (goal 1.2400), xdot(T)=0.00e+00, peak v=0.1371 m/s at t=5.31 s, xddot(0)=0.0290, xddot(T)=-0.0387 m/s^2, alpha_y=[0. 0. 0. 0.]
- d0=1.269 (final run, chase to the stop): T=10.6900 s, alpha_x=[ 0.        0.06      0.016838 -0.001225], x(T)=1.0690 (goal 1.0690), xdot(T)=0.00e+00, peak v=0.1371 m/s at t=4.58 s, xddot(0)=0.0337, xddot(T)=-0.0449 m/s^2, alpha_y=[0. 0. 0. 0.]
- d0=1.365 (run 011925, last chase): T=11.6500 s, alpha_x=[ 0.        0.06      0.015451 -0.001032], x(T)=1.1650 (goal 1.1650), xdot(T)=5.55e-17, peak v=0.1371 m/s at t=4.99 s, xddot(0)=0.0309, xddot(T)=-0.0412 m/s^2, alpha_y=[0. 0. 0. 0.]
- d0=1.375 (run 012232, last chase): T=11.7500 s, alpha_x=[ 0.        0.06      0.015319 -0.001014], x(T)=1.1750 (goal 1.1750), xdot(T)=5.55e-17, peak v=0.1371 m/s at t=5.03 s, xddot(0)=0.0306, xddot(T)=-0.0409 m/s^2, alpha_y=[0. 0. 0. 0.]
- d0=1.500 (spec lower bound): T=13.0000 s, alpha_x=[ 0.        0.06      0.013846 -0.000828], x(T)=1.3000 (goal 1.3000), xdot(T)=0.00e+00, peak v=0.1371 m/s at t=5.57 s, xddot(0)=0.0277, xddot(T)=-0.0369 m/s^2, alpha_y=[0. 0. 0. 0.]
- d0=1.800 (spec upper bound): T=16.0000 s, alpha_x=[ 0.        0.06      0.01125  -0.000547], x(T)=1.6000 (goal 1.6000), xdot(T)=0.00e+00, peak v=0.1371 m/s at t=6.86 s, xddot(0)=0.0225, xddot(T)=-0.0300 m/s^2, alpha_y=[0. 0. 0. 0.]
- Analytic peak with T = 1.6 D / vmax and xdot0 = 0.06: v_pk = 0.1371 m/s (85.7 % of vmax); rest-to-rest (xdot0 = 0): 1.5/1.6 vmax = 0.1500 m/s. T = 4 s floor applies for d0 < 0.60 m
- chase 1: t=8.60-10.45 s, d0=1.440 m, T=12.400 s, goal 1.240 m; plan x at end 0.1549 m, odometry along 0.1289 m, cross +1.8 cm, straight distance 0.1302 m
- chase 2: t=11.60-22.25 s, d0=1.269 m, T=10.690 s, goal 1.069 m; plan x at end 1.0690 m, odometry along 1.0520 m, cross -13.2 cm, straight distance 1.0602 m
  final chase tracking: along - plan: mean -3.0 cm, max |err| 4.8 cm; |cross| max 13.2 cm

## All logged runs
- dryrun/20261026_010948: dry run, \code{cmd_vel} remapped; 11.2 s; chase starts 1 at [0.15]; d0 0.999; T 7.99; ARRIVED 8.15; trigger plan time $T$; last camera d 1.000; tape None
- wheels_off_test/20261026_011222: wheels off the ground; 11.2 s; chase starts 1 at [0.15]; d0 1.010; T 8.10; ARRIVED 8.20; trigger plan time $T$; last camera d 1.011; tape None
- wheels_off_test/20261026_011631: wheels off the ground; 10.4 s; chase starts 1 at [0.15]; d0 0.924; T 7.24; ARRIVED 7.40; trigger plan time $T$; last camera d 0.405; tape None
- runs/20261026_011925: floor run 1; 20.1 s; chase starts 5 at [0.15, 2.0, 3.9, 4.6, 5.4]; d0 1.365; T 11.65; ARRIVED 17.10; trigger plan time $T$; last camera d 0.397; tape 0.21
- runs/20261026_012232: floor run 2; 16.4 s; chase starts 2 at [0.15, 2.2]; d0 1.375; T 11.75; ARRIVED 13.35; trigger odometry; last camera d 0.400; tape 0.2
- dryrun/20261026_013032: dry run, \code{cmd_vel} remapped; 58.0 s; chase starts 0 at []; d0 n/a; T n/a; ARRIVED n/a; trigger no tag in view (search only); last camera d n/a; tape None
- runs/20261026_015001: floor run 3 (final, filmed); 25.3 s; chase starts 2 at [8.6, 11.6]; d0 1.269; T 10.69; ARRIVED 22.30; trigger plan time $T$; last camera d 0.391; tape 0.2
## Final run timeline
- first detection at t = 5.40 s, bearing 0.763 rad, d 1.374 m
- heading change before first detection: 65.8 deg
- heading change at first CHASING tick: 136.2 deg
- ToF range over the run: 0.382 .. 0.865 m
- last camera d: 0.391 m; last new detection at t = 19.60 s
- odometry displacement start->end: 1.2158 m; path length 1.4204 m
- final chase start bearing -0.158 rad (-9.1 deg); end point in chase frame X=1.045 Y=-0.133 m; heading change during final chase -6.7 deg

## Problem 1: camera geometry and detection
- 720p intrinsics fx=628.58 fy=625.79 cx=638.02 cy=361.6: horizontal half FOV 45.4 / 45.6 deg
- vertical half FOV below the axis 29.8 deg; lower image edge 35.1 deg below horizon
- lower edge meets the floor 0.445 m ahead of the camera = 0.283 m from the chassis front (tag center); whole 155 mm tag visible only beyond about 0.361 m
- standoff 0.20 m from the front: ground distance from camera 0.362 m, slant range 0.479 m; slant - offset = 0.317 m (would brake 11.7 cm late)
- note example (camera 0.25 m up, 25 deg pitch, tag 0.20 m ahead): slant 0.320 m
- calibration rows (tape, horizontal camera distance, tape - camera, rate/s, final pose):
  - row 1: 1.000, 1.0231, -0.0231, 14.6, False
  - row 2: 0.600, 0.6948, -0.0948, 12.6, False
  - row 3: 1.200, 1.2991, -0.0991, 9.2, True
  - row 4: 1.500, 1.6621, -0.1621, 3.8, True
  - row 5: 0.600, 0.7675, -0.1675, 13.6, True
  - row 6: 1.000, 1.1619, -0.1619, 14.8, True
- height used for the calibration: 0.31 m; final rows (1-based): [3, 4, 5, 6]
- final-pose rows: median(tape - camera) = -0.1620 m; lstsq tape = 1.0176 cam -0.1692
- residual with the configured offset -0.162 m: +62.9 mm, -0.1 mm, -5.5 mm, +0.1 mm
- render study 720p: all yaws detected from 0.45 to 1.15 m; some detected up to 1.60 m; tag size at 1.5 m 31.2 px; mean range error (detected) 6.4 mm, max 29.7 mm
- render study 360p: all yaws detected from 0.45 to 0.75 m; some detected up to 1.00 m; tag size at 1.5 m 15.6 px; mean range error (detected) 5.4 mm, max 26.6 mm

## Driver connection logs
- driver_20261025_235452.log: mode ap, 17 connects, 16 disconnects, 27.0 min from first to last event; connected time before each drop: median 64.6 s, min 25.0 s, max 227.2 s, n = 16
- driver_20261026_002315.log: mode ap, 4 connects, 3 disconnects, 8.0 min from first to last event; connected time before each drop: median 130.1 s, min 64.1 s, max 263.2 s, n = 3
- driver_20261026_003846.log: mode ap, 4 connects, 4 disconnects, 3.7 min from first to last event; connected time before each drop: median 41.1 s, min 14.0 s, max 64.1 s, n = 4
- driver_20261026_005040.log: mode sta, 1 connects, 1 disconnects, 69.3 min from first to last event; connected time before each drop: median 4155.9 s, min 4155.9 s, max 4155.9 s, n = 1

## Media
- demo video A2_demo_side_by_side.mp4: 1920x480, 30.0 fps, 23.8 s
- still searching: t = 3.0 s
- still aligning: t = 5.5 s
- still realign: t = 11.0 s
- still chasing: t = 14.0 s
- still arrived: t = 23.5 s

## Unit tests (ros2/rm_flatness_nav/test)
- pytest: 36 passed in 0.65s

## Closed-loop mission simulation (test_mission.simulate)
- tag (1.6, 0.0), start heading 0 deg: final state ARRIVED, distance to tag 0.200 m, peak v 0.1371 m/s, brake: odometry 1.400 m covered
- tag (1.8, 0.25), start heading 0 deg: final state ARRIVED, distance to tag 0.200 m, peak v 0.1371 m/s, brake: odometry 1.617 m covered
- tag (1.5, -0.3), start heading 0 deg: final state ARRIVED, distance to tag 0.200 m, peak v 0.1371 m/s, brake: odometry 1.330 m covered
- tag (1.6, 0.0), start heading -90 deg: final state ARRIVED, distance to tag 0.200 m, peak v 0.1371 m/s, brake: odometry 1.400 m covered

## Figures
- `report/figures/p3_v_V1.pdf`
- `report/figures/p3_voltages_all.pdf`
- `report/figures/p2_profiles.pdf`
- `report/figures/p2_plan_vs_odom.pdf`
- `report/figures/p5_timeline.pdf`
- `report/figures/p5_topview.pdf`
- `report/figures/p1_range.pdf`
- `report/figures/photo_arm_side.jpg`
- `report/figures/photo_front_tof.jpg`
- `report/figures/photo_top_view.jpg`
- `report/figures/still_searching.jpg`
- `report/figures/still_aligning.jpg`
- `report/figures/still_realign.jpg`
- `report/figures/still_chasing.jpg`
- `report/figures/still_arrived.jpg`
