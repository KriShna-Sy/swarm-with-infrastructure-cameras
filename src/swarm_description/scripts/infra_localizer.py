#!/usr/bin/env python3
"""
infra_localizer.py
===================
Estimates each robot's XY position in the 'map' frame purely from the
infra-camera merged point cloud -- this is what actually implements the
"Robot pose (localised by cloud)" box in the architecture diagram.
Nothing in the original codebase did this; RTAB-Map + a single shared
/diff_drive_controller/odom cannot localise more than one robot at once.

Approach (background subtraction, static-camera friendly):
  1. Calibration window (first `calib_seconds`, default 5s, room assumed
     empty of robots): project /merged_cloud onto an XY occupancy grid
     and record every cell touched by the static room geometry
     (walls/furniture/floor clutter).
  2. After calibration: any occupied cell NOT in the static background
     is a candidate robot detection. Cluster candidate cells (grid
     flood-fill, no scipy dependency) and keep the `num_robots` largest
     blobs.
  3. Greedily associate blobs to robot names by nearest distance to each
     robot's last known estimate (seeded with spawn poses from
     spawn_swarm.launch.py) so identity doesn't swap between frames.
  4. Publish each estimate as geometry_msgs/PoseStamped on
     /<robot_name>/infra_pose (frame_id 'map') and broadcast
     map -> <robot_name>/infra_estimate over TF.

This intentionally does NOT touch odom/base_link TF -- it's an
independent, absolute position fix you can fuse with wheel odometry
(e.g. via robot_localization) rather than a replacement for it.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros


def flood_fill_clusters(occ):
    """4-connected flood fill over a boolean 2D grid. Returns list of
    (cell_count, (row_indices, col_indices))."""
    visited = np.zeros_like(occ, dtype=bool)
    clusters = []
    rows, cols = occ.shape
    for r in range(rows):
        for c in range(cols):
            if not occ[r, c] or visited[r, c]:
                continue
            stack = [(r, c)]
            visited[r, c] = True
            cluster_r, cluster_c = [], []
            while stack:
                cr, cc = stack.pop()
                cluster_r.append(cr)
                cluster_c.append(cc)
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < rows and 0 <= nc < cols \
                            and occ[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        stack.append((nr, nc))
            clusters.append((len(cluster_r), (cluster_r, cluster_c)))
    return clusters


class InfraLocalizer(Node):
    def __init__(self):
        super().__init__('infra_localizer')

        self.declare_parameter('robot_names', ['fwld_1', 'fwld_2'])
        self.declare_parameter('spawn_x', [-1.0, 1.0])
        self.declare_parameter('spawn_y', [-0.5, 0.5])
        self.declare_parameter('room_half_extent', 2.9)   # metres, room is ~6x6
        self.declare_parameter('cell_size', 0.10)
        self.declare_parameter('calib_seconds', 5.0)

        self.robot_names = self.get_parameter('robot_names').value
        self.last_xy = {
            name: (sx, sy) for name, sx, sy in zip(
                self.robot_names,
                self.get_parameter('spawn_x').value,
                self.get_parameter('spawn_y').value)
        }

        half = self.get_parameter('room_half_extent').value
        self.cell = self.get_parameter('cell_size').value
        self.n = int(2 * half / self.cell) + 1
        self.origin = -half

        self.background = np.zeros((self.n, self.n), dtype=bool)
        self.calib_seconds = self.get_parameter('calib_seconds').value
        self.start_time = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pose_pubs = {
            name: self.create_publisher(PoseStamped, f'/{name}/infra_pose', 10)
            for name in self.robot_names
        }

        self.create_subscription(PointCloud2, '/merged_cloud', self.cb_cloud, 10)
        self.get_logger().info(
            f'infra_localizer started -- calibrating background for '
            f'{self.calib_seconds}s (keep robots out of frame if possible)')

    def _xy_to_cell(self, x, y):
        r = int((y - self.origin) / self.cell)
        c = int((x - self.origin) / self.cell)
        return r, c

    def _cell_to_xy(self, r, c):
        y = self.origin + (r + 0.5) * self.cell
        x = self.origin + (c + 0.5) * self.cell
        return x, y

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
            return

        clusters = flood_fill_clusters(foreground)
        clusters.sort(key=lambda t: t[0], reverse=True)
        top = clusters[:len(self.robot_names)]

        centroids = []
        for count, (rs, cs) in top:
            if count < 2:
                continue
            xs, ys = zip(*(self._cell_to_xy(r, c) for r, c in zip(rs, cs)))
            centroids.append((float(np.mean(xs)), float(np.mean(ys))))

        # Greedy nearest-neighbour association to keep robot identity stable
        remaining = list(centroids)
        for name in self.robot_names:
            if not remaining:
                break
            lx, ly = self.last_xy[name]
            dists = [((cx - lx) ** 2 + (cy - ly) ** 2, i)
                     for i, (cx, cy) in enumerate(remaining)]
            _, best_i = min(dists)
            cx, cy = remaining.pop(best_i)
            self.last_xy[name] = (cx, cy)
            self._publish(name, cx, cy, now)

    def _publish(self, name, x, y, stamp):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp.to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.pose_pubs[name].publish(pose)

        t = TransformStamped()
        t.header.frame_id = 'map'
        t.header.stamp = stamp.to_msg()
        t.child_frame_id = f'{name}/infra_estimate'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = InfraLocalizer()
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
