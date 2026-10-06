#!/usr/bin/env python3
"""
csi_ekf_localizer.py
====================
Covariance-Steered Infrastructure Extended Kalman Filter (CSI-EKF).
Designed for publication in IEEE Applied Sensing Conference (APSCON).

Novel Aspects:
  1. Multi-Sensor Continuous-Discrete EKF: Fuses high-rate wheel odometry
     kinematics (50 Hz) with overhead infrastructure RGB-D point cloud detections (10 Hz).
  2. Range-Adaptive Measurement Covariance: Dynamically scales R(d) with distance-squared
     from ceiling cameras, modeling optical triangulation depth noise.
  3. Mahalanobis-Distance Data Association with Chi-Square Gating: Prevents identity
     swapping during agent crossovers.
  4. Non-Line-of-Sight (NLOS) Occlusion Handling: Seamlessly coasts on process model
     when obstacles (e.g. divider, pillars) occlude line of sight, with rigorous
     covariance propagation.
  5. Live Uncertainty Ellipse Visualization: Publishes covariance markers (3-sigma)
     to RViz for visual analytics.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


def flood_fill_clusters(occ):
    """Connected component labeling on 2D occupancy grid."""
    visited = np.zeros_like(occ, dtype=bool)
    clusters = []
    rows, cols = occ.shape
    for r in range(rows):
        for c in range(cols):
            if not occ[r, c] or visited[r, c]:
                continue
            stack = [(r, c)]
            visited[r, c] = True
            cr_list, cc_list = [], []
            while stack:
                cr, cc = stack.pop()
                cr_list.append(cr)
                cc_list.append(cc)
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < rows and 0 <= nc < cols and occ[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        stack.append((nr, nc))
            clusters.append((len(cr_list), (cr_list, cc_list)))
    return clusters


class RobotEKFTrack:
    """Represents a continuous-discrete EKF state for one swarm agent."""
    def __init__(self, name, init_x, init_y, init_yaw=0.0):
        self.name = name
        # State: [x, y, yaw, v, omega]
        self.x = np.array([init_x, init_y, init_yaw, 0.0, 0.0], dtype=np.float64)
        
        # State Covariance P (5x5)
        self.P = np.diag([0.04, 0.04, 0.10, 0.05, 0.05])
        
        # Process Noise Q (5x5)
        self.Q = np.diag([1e-4, 1e-4, 5e-4, 1e-2, 5e-2])
        
        self.last_odom_t = None
        self.last_update_t = None
        self.occluded = False
        self.consecutive_misses = 0

    def predict_from_odometry(self, v_odom, omega_odom, dt):
        """Kinematic motion update driven by onboard odometry velocity."""
        if dt <= 0.0 or dt > 1.0:
            return

        theta = self.x[2]
        # State propagation (unicycle model)
        self.x[0] += v_odom * math.cos(theta) * dt
        self.x[1] += v_odom * math.sin(theta) * dt
        self.x[2] = (self.x[2] + omega_odom * dt + math.pi) % (2 * math.pi) - math.pi
        self.x[3] = v_odom
        self.x[4] = omega_odom

        # Jacobian F_x
        F = np.eye(5)
        F[0, 2] = -v_odom * math.sin(theta) * dt
        F[0, 3] = math.cos(theta) * dt
        F[1, 2] = v_odom * math.cos(theta) * dt
        F[1, 3] = math.sin(theta) * dt
        F[2, 4] = dt

        # Covariance update: P = F P F^T + Q * dt
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_measurement(self, z_x, z_y, R_cov):
        """Measurement update from infrastructure camera detection."""
        # Measurement matrix H (2x5) observing [x, y]
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        z = np.array([z_x, z_y])
        z_hat = H @ self.x
        y = z - z_hat  # Innovation

        # Innovation covariance S = H P H^T + R
        S = H @ self.P @ H.T + R_cov
        S_inv = np.linalg.inv(S)

        # Kalman gain K = P H^T S^-1
        K = self.P @ H.T @ S_inv

        # State correction
        self.x += K @ y
        self.x[2] = (self.x[2] + math.pi) % (2 * math.pi) - math.pi

        # Joseph form covariance update for numerical stability: P = (I - KH)P(I - KH)^T + KRK^T
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_cov @ K.T
        self.occluded = False
        self.consecutive_misses = 0

    def compute_mahalanobis_dist(self, z_x, z_y, R_cov):
        """Calculates Mahalanobis distance squared d_M^2 for data association."""
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        z = np.array([z_x, z_y])
        y = z - (H @ self.x)
        S = H @ self.P @ H.T + R_cov
        try:
            S_inv = np.linalg.inv(S)
            return float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            return float('inf')


class CSIEKFLocalizer(Node):
    def __init__(self):
        super().__init__('csi_ekf_localizer')

        self.declare_parameter('robot_names', ['fwld_1', 'fwld_2'])
        self.declare_parameter('spawn_x', [-1.0, 1.0])
        self.declare_parameter('spawn_y', [-0.5, 0.5])
        self.declare_parameter('room_half_extent', 3.0)
        self.declare_parameter('cell_size', 0.08)
        self.declare_parameter('calib_seconds', 4.0)
        self.declare_parameter('gate_threshold', 9.21)  # Chi-square 2-DoF 99% confidence gate

        self.robot_names = self.get_parameter('robot_names').value
        sp_x = self.get_parameter('spawn_x').value
        sp_y = self.get_parameter('spawn_y').value

        self.tracks = {}
        for name, sx, sy in zip(self.robot_names, sp_x, sp_y):
            self.tracks[name] = RobotEKFTrack(name, float(sx), float(sy))

        half = float(self.get_parameter('room_half_extent').value)
        self.cell = float(self.get_parameter('cell_size').value)
        self.n = int(2 * half / self.cell) + 1
        self.origin = -half
        self.gate_thresh = float(self.get_parameter('gate_threshold').value)

        self.background = np.zeros((self.n, self.n), dtype=bool)
        self.calib_seconds = float(self.get_parameter('calib_seconds').value)
        self.start_time = None

        # Camera optical poses in world for adaptive distance-covariance calculation
        self.cam_oak_pos = np.array([-2.8, -2.8, 2.0])
        self.cam_rs_pos = np.array([2.8, 2.8, 2.0])

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self.ekf_pubs = {
            name: self.create_publisher(PoseWithCovarianceStamped, f'/{name}/ekf_pose', 10)
            for name in self.robot_names
        }
        self.ellipse_pub = self.create_publisher(MarkerArray, '/swarm/uncertainty_ellipses', 10)

        # Subscriptions
        self.create_subscription(PointCloud2, '/merged_cloud', self.cb_cloud, 10)
        for name in self.robot_names:
            self.create_subscription(
                Odometry, f'/model/{name}/odometry',
                lambda msg, n=name: self.cb_odom(msg, n), 10)

        # High-rate state publishing & TF broadcast at 30 Hz
        self.timer = self.create_timer(0.033, self.publish_states_and_tf)
        self.get_logger().info('CSI-EKF Localizer initialized (Asymmetric Multi-View Information EKF)')

    def cb_odom(self, msg, robot_name):
        track = self.tracks.get(robot_name)
        if track is None:
            return

        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if track.last_odom_t is None:
            track.last_odom_t = t_sec
            return

        dt = t_sec - track.last_odom_t
        track.last_odom_t = t_sec

        # Extract forward velocity and angular rate
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        track.predict_from_odometry(v, omega, dt)

    def _xy_to_cell(self, x, y):
        return int((y - self.origin) / self.cell), int((x - self.origin) / self.cell)

    def _cell_to_xy(self, r, c):
        return self.origin + (c + 0.5) * self.cell, self.origin + (r + 0.5) * self.cell

    def _get_adaptive_measurement_cov(self, x, y):
        """Computes distance-squared adaptive measurement covariance R(d)."""
        p = np.array([x, y, 0.05])
        d_oak = np.linalg.norm(p - self.cam_oak_pos)
        d_rs = np.linalg.norm(p - self.cam_rs_pos)
        d_min = min(d_oak, d_rs)

        # Base noise 1.5 cm, growing quadratically with camera distance
        sigma_base = 0.015
        sigma_sq = sigma_base**2 + 0.001 * (d_min ** 2)
        return np.diag([sigma_sq, sigma_sq])

    def cb_cloud(self, msg):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        elapsed = (now - self.start_time).nanoseconds / 1e9

        occ = np.zeros((self.n, self.n), dtype=bool)
        for x, y, z in pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True):
            if not (np.isfinite(x) and np.isfinite(y)):
                continue
            r, c = self._xy_to_cell(x, y)
            if 0 <= r < self.n and 0 <= c < self.n:
                occ[r, c] = True

        if elapsed < self.calib_seconds:
            self.background |= occ
            return

        foreground = occ & ~self.background
        if not foreground.any():
            for track in self.tracks.values():
                track.consecutive_misses += 1
                if track.consecutive_misses > 3:
                    track.occluded = True
            return

        clusters = flood_fill_clusters(foreground)
        clusters.sort(key=lambda t: t[0], reverse=True)

        candidate_centroids = []
        for count, (rs, cs) in clusters:
            if count < 2:
                continue
            xs, ys = zip(*(self._cell_to_xy(r, c) for r, c in zip(rs, cs)))
            candidate_centroids.append((float(np.mean(xs)), float(np.mean(ys))))

        if not candidate_centroids:
            return

        # Probabilistic Gated Data Association (Mahalanobis Distance)
        unmatched_tracks = set(self.tracks.keys())
        for zx, zy in candidate_centroids:
            if not unmatched_tracks:
                break
            R_cov = self._get_adaptive_measurement_cov(zx, zy)
            best_robot = None
            min_dist = float('inf')

            for name in unmatched_tracks:
                track = self.tracks[name]
                d_m2 = track.compute_mahalanobis_dist(zx, zy, R_cov)
                if d_m2 < self.gate_thresh and d_m2 < min_dist:
                    min_dist = d_m2
                    best_robot = name

            if best_robot is not None:
                self.tracks[best_robot].update_measurement(zx, zy, R_cov)
                unmatched_tracks.remove(best_robot)

        for name in unmatched_tracks:
            self.tracks[name].consecutive_misses += 1
            if self.tracks[name].consecutive_misses > 3:
                self.tracks[name].occluded = True

    def publish_states_and_tf(self):
        now_msg = self.get_clock().now().to_msg()
        marker_arr = MarkerArray()
        m_id = 0

        for name, track in self.tracks.items():
            # 1. Publish PoseWithCovarianceStamped
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = now_msg
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = float(track.x[0])
            msg.pose.pose.position.y = float(track.x[1])
            msg.pose.pose.position.z = 0.036

            half_yaw = track.x[2] * 0.5
            msg.pose.pose.orientation.z = math.sin(half_yaw)
            msg.pose.pose.orientation.w = math.cos(half_yaw)

            # Pack 6x6 covariance (fill X, Y, Yaw elements)
            cov6 = [0.0] * 36
            cov6[0] = float(track.P[0, 0])
            cov6[1] = float(track.P[0, 1])
            cov6[6] = float(track.P[1, 0])
            cov6[7] = float(track.P[1, 1])
            cov6[35] = float(track.P[2, 2])
            msg.pose.covariance = cov6
            self.ekf_pubs[name].publish(msg)

            # 2. Broadcast TF map -> {name}_ekf_base_link
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_msg
            tf_msg.header.frame_id = 'map'
            tf_msg.child_frame_id = f'{name}_ekf_base'
            tf_msg.transform.translation.x = float(track.x[0])
            tf_msg.transform.translation.y = float(track.x[1])
            tf_msg.transform.translation.z = 0.036
            tf_msg.transform.rotation.z = math.sin(half_yaw)
            tf_msg.transform.rotation.w = math.cos(half_yaw)
            self.tf_broadcaster.sendTransform(tf_msg)

            # 3. RViz 3-Sigma Covariance Uncertainty Ellipse Marker
            sub_P = track.P[:2, :2]
            vals, vecs = np.linalg.eigh(sub_P)
            order = vals.argsort()[::-1]
            vals = vals[order]
            vecs = vecs[:, order]
            phi = math.atan2(vecs[1, 0], vecs[0, 0])

            # 3-sigma semi-axes
            sigma_x = 3.0 * math.sqrt(max(1e-6, vals[0]))
            sigma_y = 3.0 * math.sqrt(max(1e-6, vals[1]))

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now_msg
            m.ns = f'{name}_covariance'
            m.id = m_id
            m_id += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(track.x[0])
            m.pose.position.y = float(track.x[1])
            m.pose.position.z = 0.01

            half_phi = phi * 0.5
            m.pose.orientation.z = math.sin(half_phi)
            m.pose.orientation.w = math.cos(half_phi)

            m.scale.x = float(sigma_x * 2.0)
            m.scale.y = float(sigma_y * 2.0)
            m.scale.z = 0.005

            if track.occluded:
                m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.0; m.color.a = 0.45  # Orange/Red when occluded
            else:
                m.color.r = 0.0; m.color.g = 0.8; m.color.b = 1.0; m.color.a = 0.35  # Cyan when directly tracked
            marker_arr.markers.append(m)

        self.ellipse_pub.publish(marker_arr)


def main():
    rclpy.init()
    node = CSIEKFLocalizer()
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
