# SCE 672 Assessment 2: Differential Flatness Pursuit and Motor Voltage Dynamics

Ziad Aboelkhir (g202622300), KFUPM, Prof. Aamer Iqbal Bhatti. DJI RoboMaster EP Core, ROS 2 Jazzy.

The robot sweeps in place until it sees AprilTag T3 on the floor, centers it, follows a cubic flat-output
trajectory toward it and stops 20 cm before it, while logging the four chassis motor voltages every second.

## Deliverables
| Item | Location |
|---|---|
| ROS 2 package | [`ros2/rm_flatness_nav/`](ros2/rm_flatness_nav/) (see its README) |
| Motor voltage log (final run, Table 1) | [`motor_voltage_log.csv`](motor_voltage_log.csv) (copy of `data/runs/20261026_015001/motor_voltage_log.csv`) |
| Report | `report/` (LaTeX, Overleaf) |
| Demo video | link in the report (phone view + robot view side by side, 23.8 s) |

## Layout
- `ros2/rm_flatness_nav/`: package (symlinked into `~/ros2_ws/src`)
- `data/runs/`: real runs (each with `notes.md` and the tape-measured standoff), `data/calibration/`: range calibration,
  `data/wheels_off_test/`, `data/dryrun/`, `data/logs/`: raw driver and detector logs
- `tools/`: helpers (router-mode QR code, arm step with stall guard, range probe, live view recorder, replay renderer)
- `analysis/`: scripts that build the report figures from `data/`
- `notes/`: tag notes, setup drawing, robot photos
- `docs/`: assignment and printed tag sheet
