#!/usr/bin/env python3
"""
swarm_research_evaluator.py
===========================
Real-time Quantitative Benchmark Evaluator for IEEE APSCON 2026.

Collects ground truth and estimates to generate experimental metrics:
  1. Localization RMSE (m) comparing Baseline vs CSI-EKF against Gazebo Ground Truth.
  2. Safety Clearance Analysis: Minimum inter-agent distance and barrier compliance.
  3. Sensing Quality Metrics: Trace of Covariance Tr(P) during line-of-sight vs occlusion.
  4. Automatic CSV export for publication plotting.
"""
import os
import math
import csv
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String


class SwarmResearchEvaluator(Node):
    def __init__(self):
        super().__init__('swarm_research_evaluator')

        self.declare_parameter('robot_names', ['fwld_1', 'fwld_2'])
        self.declare_parameter('csv_output_dir', os.path.expanduser('~/swarm2_ws_fixed'))
        self.declare_parameter('eval_period_s', 1.0)

        self.robot_names = self.get_parameter('robot_names').value
        self.csv_dir = self.get_parameter('csv_output_dir').value
        self.csv_path = os.path.join(self.csv_dir, 'apscon_benchmark_results.csv')

        # Real-time state cache
        self.gt_poses = {}      # Ground truth from Gazebo physics
        self.ekf_poses = {}     # Proposed CSI-EKF
        self.ekf_cov_tr = {}    # Tr(P)
        self.naive_poses = {}   # Baseline naive cluster

        # Statistical accumulators
        self.sq_err_ekf = {name: [] for name in self.robot_names}
        self.sq_err_naive = {name: [] for name in self.robot_names}
        self.min_dist = float('inf')
        self.safety_violations = 0
        self.total_samples = 0

        # Subscriptions
        for name in self.robot_names:
            # Ground truth from model odometry
            self.create_subscription(
                Odometry, f'/model/{name}/odometry',
                lambda msg, n=name: self.cb_gt(msg, n), 10)

            # Proposed CSI-EKF
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{name}/ekf_pose',
                lambda msg, n=name: self.cb_ekf(msg, n), 10)

            # Baseline naive cluster
            self.create_subscription(
                PoseStamped, f'/{name}/infra_pose',
                lambda msg, n=name: self.cb_naive(msg, n), 10)

        # Periodic logging and benchmarking timer
        period = float(self.get_parameter('eval_period_s').value)
        self.timer = self.create_timer(period, self.evaluate_and_log)

        # Initialize CSV file with headers
        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp_s',
                    'rmse_ekf_fwld_1', 'rmse_naive_fwld_1', 'cov_trace_fwld_1',
                    'rmse_ekf_fwld_2', 'rmse_naive_fwld_2', 'cov_trace_fwld_2',
                    'inter_robot_dist', 'min_clearance_record'
                ])
            self.get_logger().info(f'Benchmark logger ready. Exporting metrics to: {self.csv_path}')
        except Exception as e:
            self.get_logger().warn(f'Could not initialize CSV: {e}')

    def cb_gt(self, msg, name):
        # Ground truth pose in map coordinates (fused with spawn offset)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # fwld_1 spawned at (-1.0, -0.5), fwld_2 at (1.0, 0.5)
        offset_x = -1.0 if name == 'fwld_1' else 1.0
        offset_y = -0.5 if name == 'fwld_1' else 0.5
        self.gt_poses[name] = (x + offset_x, y + offset_y)

    def cb_ekf(self, msg, name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.ekf_poses[name] = (x, y)
        cov = msg.pose.covariance
        self.ekf_cov_tr[name] = float(cov[0] + cov[7])

    def cb_naive(self, msg, name):
        self.naive_poses[name] = (msg.pose.position.x, msg.pose.position.y)

    def evaluate_and_log(self):
        if len(self.gt_poses) < 2 or len(self.ekf_poses) < 2:
            return

        now = self.get_clock().now()
        t_sec = now.nanoseconds / 1e9
        self.total_samples += 1

        # 1. Compute Errors
        rmse_ekf_vals = {}
        rmse_naive_vals = {}
        for name in self.robot_names:
            gt = self.gt_poses.get(name)
            ekf = self.ekf_poses.get(name)
            naive = self.naive_poses.get(name)

            if gt and ekf:
                err2 = (ekf[0] - gt[0])**2 + (ekf[1] - gt[1])**2
                self.sq_err_ekf[name].append(err2)
                rmse_ekf_vals[name] = math.sqrt(np.mean(self.sq_err_ekf[name]))
            else:
                rmse_ekf_vals[name] = 0.0

            if gt and naive:
                err2_n = (naive[0] - gt[0])**2 + (naive[1] - gt[1])**2
                self.sq_err_naive[name].append(err2_n)
                rmse_naive_vals[name] = math.sqrt(np.mean(self.sq_err_naive[name]))
            else:
                rmse_naive_vals[name] = 0.0

        # 2. Inter-agent clearance
        p1 = self.gt_poses['fwld_1']
        p2 = self.gt_poses['fwld_2']
        dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
        if dist < self.min_dist:
            self.min_dist = dist
        if dist < 0.40:  # Collision threshold: 2 * 0.20m radius
            self.safety_violations += 1

        # 3. Log formatted benchmarking table
        tr1 = self.ekf_cov_tr.get('fwld_1', 0.0)
        tr2 = self.ekf_cov_tr.get('fwld_2', 0.0)

        self.get_logger().info(
            f'\n╔════════════════════════════════════════════════════════════════════════╗\n'
            f'║  IEEE APSCON BENCHMARK: CSI-EKF vs BASELINE (t = {t_sec:.1f}s)             ║\n'
            f'╠════════════════════════════════════════════════════════════════════════╣\n'
            f'║ fwld_1 -> Proposed EKF RMSE: {rmse_ekf_vals.get("fwld_1",0.0):.4f} m | Cov Trace: {tr1:.5f} m² ║\n'
            f'║ fwld_2 -> Proposed EKF RMSE: {rmse_ekf_vals.get("fwld_2",0.0):.4f} m | Cov Trace: {tr2:.5f} m² ║\n'
            f'║ Current Clearance: {dist:.3f} m | Min Clearance: {self.min_dist:.3f} m (Safe >= 0.40m)     ║\n'
            f'╚════════════════════════════════════════════════════════════════════════╝'
        )

        # 4. Save to CSV
        try:
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f'{t_sec:.2f}',
                    f'{rmse_ekf_vals.get("fwld_1",0.0):.5f}',
                    f'{rmse_naive_vals.get("fwld_1",0.0):.5f}',
                    f'{tr1:.6f}',
                    f'{rmse_ekf_vals.get("fwld_2",0.0):.5f}',
                    f'{rmse_naive_vals.get("fwld_2",0.0):.5f}',
                    f'{tr2:.6f}',
                    f'{dist:.4f}',
                    f'{self.min_dist:.4f}'
                ])
        except Exception:
            pass


def main():
    rclpy.init()
    node = SwarmResearchEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
