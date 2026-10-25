# SCE 672 Assessment 2: Differential Flatness Pursuit and Motor Voltage Dynamics

Ziad Aboelkhir (g202622300), KFUPM, Prof. Aamer Iqbal Bhatti.

DJI RoboMaster EP Core, ROS 2 Jazzy. The robot searches for AprilTag T3 on the floor, centers it,
follows a cubic flat-output trajectory, stops at a 20 cm standoff and logs motor voltages V1..V4.

## Layout
- `ros2/rm_flatness_nav/`: ROS 2 package (symlinked into `~/ros2_ws/src`)
- `data/`: measured runs, including `motor_voltage_log.csv`
- `analysis/`: scripts that turn `data/` into figures and tables
- `report/`: LaTeX report
- `video/`: demo video link
- `docs/`: assignment specification
