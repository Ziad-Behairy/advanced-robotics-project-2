"""Planner node (A2 Problems 1-3): mission state machine, flat-output pursuit, brake and voltage logging.

- Inputs: tag/range_bearing (tag_detector), range_0 (front ToF), odom.
- Output: cmd_vel at control_rate. Every exit path (normal stop, Ctrl+C, exception) sends zero velocity.
- Every control tick the commanded (v, omega) is mapped to wheel rates (eq. 4) and armature voltages (eq. 5),
  with dt = the control period. Every log_period seconds one row is printed and written to
  motor_voltage_log.csv; every tick is written to motor_voltage_full.csv (for the v(t), V1(t) plot).
"""
import csv
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, String

from rm_flatness_nav.mission import ARRIVED, Mission, MissionParams
from rm_flatness_nav.voltage import VoltageModel


class Planner(Node):

    def __init__(self):
        super().__init__('flatness_planner')
        self.rate = self.declare_parameter('control_rate', 20.0).value
        self.log_period = self.declare_parameter('log_period', 1.0).value
        self.after_arrival = self.declare_parameter('log_after_arrival', 3.0).value  # s of 0 V rows
        out_dir = os.path.expanduser(self.declare_parameter('output_dir', '~/advanced_robotics_project_2/data/runs').value)
        search_dir = self.declare_parameter('search_dir', 1.0).value  # +1 CCW (left) when never seen

        self.mission = Mission(MissionParams(), search_dir=search_dir)
        self.volts = VoltageModel()
        self.dt = 1.0 / self.rate
        self.odom_xy = None

        run = time.strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(out_dir, run)
        os.makedirs(self.run_dir, exist_ok=True)
        self.log_file = open(os.path.join(self.run_dir, 'motor_voltage_log.csv'), 'w', newline='')
        self.full_file = open(os.path.join(self.run_dir, 'motor_voltage_full.csv'), 'w', newline='')
        self.log = csv.writer(self.log_file)
        self.full = csv.writer(self.full_file)
        header = ['t_s', 'V1', 'V2', 'V3', 'V4', 'state', 'v_cmd', 'w_cmd', 'w1', 'w2', 'w3', 'w4',
                  'tag_d', 'tag_bearing', 'tof']
        self.log.writerow(header)
        self.full.writerow(header)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'mission/state', 10)
        self.create_subscription(Float32MultiArray, 'tag/range_bearing', self.on_tag, 10)
        self.create_subscription(Range, 'range_0', self.on_tof, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        self.t0 = self.now()
        self.next_log = 0.0
        self.arrived_at = None
        self.done = False
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'Run folder: {self.run_dir}')
        print(f"{'t (s)':>6} {'V1 (V)':>8} {'V2 (V)':>8} {'V3 (V)':>8} {'V4 (V)':>8}  state", flush=True)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_tag(self, msg):
        d, bearing = float(msg.data[0]), float(msg.data[1])
        self.mission.on_tag(self.now(), d, bearing)

    def on_tof(self, msg):
        self.mission.on_tof(float(msg.range))

    def on_odom(self, msg):
        self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def tick(self):
        if self.done:
            return
        now = self.now()
        prev_state = self.mission.state
        v, w = self.mission.step(now, self.odom_xy)
        self.send(v, w)
        if self.mission.state != prev_state:
            extra = f' ({self.mission.brake_reason})' if self.mission.state == ARRIVED else ''
            if self.mission.d0 is not None and prev_state != 'CHASING' and self.mission.state == 'CHASING':
                extra = f' (d0={self.mission.d0:.3f} m, T={self.mission.traj.T:.1f} s)'
            self.get_logger().info(f'{prev_state} -> {self.mission.state}{extra}')
        self.state_pub.publish(String(data=self.mission.state))

        wheels, volts = self.volts.update(v, w, self.dt)
        t = now - self.t0
        m = self.mission
        row = [f'{t:.3f}', *(f'{x:.3f}' for x in volts), m.state, f'{v:.4f}', f'{w:.4f}',
               *(f'{x:.3f}' for x in wheels),
               '' if m.distance is None else f'{m.distance:.3f}', f'{m.bearing:.3f}',
               '' if m.tof is None else f'{m.tof:.3f}']
        self.full.writerow(row)
        if t + 1e-6 >= self.next_log:
            self.log.writerow([f'{round(t):d}', *row[1:]])
            self.log_file.flush()
            print(f'{round(t):>6d} {volts[0]:>8.3f} {volts[1]:>8.3f} {volts[2]:>8.3f} {volts[3]:>8.3f}  {m.state}',
                  flush=True)
            self.next_log += self.log_period

        if m.state == ARRIVED:
            if self.arrived_at is None:
                self.arrived_at = now
            elif now - self.arrived_at >= self.after_arrival:
                self.finish()

    def send(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def finish(self):
        self.done = True
        for _ in range(5):
            self.send(0.0, 0.0)
        self.log_file.close()
        self.full_file.close()
        self.get_logger().info(f'Mission complete. Logs in {self.run_dir}')

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
        if not self.log_file.closed:
            self.log_file.close()
            self.full_file.close()


def main():
    # Keep the ROS context alive on Ctrl+C so the zero-velocity commands below can still be published
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Planner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()          # zero velocity on every exit path
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
