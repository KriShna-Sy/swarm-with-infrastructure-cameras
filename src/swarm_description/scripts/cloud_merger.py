#!/usr/bin/env python3
"""
cloud_merger.py
================
Merges /infra/cam_oak/depth/points and /infra/cam_rs/depth/points into a
single /merged_cloud in the 'map' frame.

Each incoming cloud arrives in its own camera optical frame (e.g.
'infra_cam_oak/link/cam_oak_rgbd'). An earlier version just relabelled
header.frame_id to 'map' WITHOUT transforming point coordinates -- since
the two cameras sit in opposite corners at different orientations, that
fused two clouds as if they were coincident, producing garbage geometry.

This version looks up the real TF and applies it -- but does the
transform manually with numpy rather than via
tf2_sensor_msgs.do_transform_cloud, which throws:
    AssertionError: PointFields and structured NumPy array dtype do not
    match for all fields!
on the point cloud ros_gz_bridge produces from Gazebo's PointCloudPacked
(its field layout/padding doesn't match what tf2_sensor_msgs expects).
Extracting xyz via sensor_msgs_py (field-name based, robust to layout)
and rotating/translating with plain numpy sidesteps that entirely.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

import tf2_ros

TARGET_FRAME = 'map'


def quat_to_rotmat(x, y, z, w):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1 - (yy + zz), xy - wz,       xz + wy],
        [xy + wz,       1 - (xx + zz), yz - wx],
        [xz - wy,       yz + wx,       1 - (xx + yy)],
    ])


class CloudMerger(Node):
    def __init__(self):
        super().__init__('cloud_merger')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cloud_oak = None
        self.cloud_rs = None

        self.create_subscription(
            PointCloud2, '/infra/cam_oak/depth/points', self.cb_oak, 10)
        self.create_subscription(
            PointCloud2, '/infra/cam_rs/depth/points', self.cb_rs, 10)

        self.pub = self.create_publisher(PointCloud2, '/merged_cloud', 10)
        self.create_timer(0.1, self.merge_and_publish)
        self.get_logger().info('cloud_merger started (TF-correct, numpy-transform version)')

    def cb_oak(self, msg):
        self.cloud_oak = msg

    def cb_rs(self, msg):
        self.cloud_rs = msg

    def _points_in_map(self, cloud):
        """Return Nx3 numpy array of this cloud's points transformed into
        TARGET_FRAME, or None if TF or the cloud isn't available."""
        if cloud is None or (cloud.width * cloud.height == 0):
            return None

        # Extract XYZ coordinates safely across all ROS 2 / NumPy array layouts
        try:
            pts_raw = pc2.read_points(cloud, field_names=['x', 'y', 'z'], skip_nans=True)
            if isinstance(pts_raw, np.ndarray):
                if pts_raw.size == 0:
                    return None
                if pts_raw.dtype.names is not None:
                    pts = np.column_stack([pts_raw['x'], pts_raw['y'], pts_raw['z']]).astype(np.float64)
                else:
                    pts = pts_raw.reshape(-1, 3).astype(np.float64)
            else:
                pts_list = list(pts_raw)
                if not pts_list:
                    return None
                pts = np.asarray(pts_list, dtype=np.float64)
                if pts.ndim == 2 and pts.shape[1] >= 3:
                    pts = pts[:, :3]
                else:
                    return None
        except Exception as e:
            self.get_logger().warn(f'Failed to extract points from cloud: {e}', throttle_duration_sec=2.0)
            return None

        if pts.size == 0:
            return None

        # Filter out NaN and Inf values
        valid_mask = np.isfinite(pts).all(axis=1)
        pts = pts[valid_mask]
        if pts.shape[0] == 0:
            return None

        # Downsample points by stride to keep DDS lightweight and responsive
        if pts.shape[0] > 15000:
            pts = pts[::4]

        try:
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME,
                cloud.header.frame_id,
                rclpy.time.Time(),          # latest available
                timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF {cloud.header.frame_id} -> {TARGET_FRAME} unavailable: {e}',
                throttle_duration_sec=2.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rotmat(q.x, q.y, q.z, q.w)
        return pts @ R.T + np.array([t.x, t.y, t.z])

    def merge_and_publish(self):
        oak_pts = self._points_in_map(self.cloud_oak)
        rs_pts = self._points_in_map(self.cloud_rs)

        arrays = [p for p in (oak_pts, rs_pts) if p is not None]
        if not arrays:
            return
        merged_pts = np.concatenate(arrays, axis=0)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = TARGET_FRAME

        # Use float32 numpy array directly; avoid Python list-of-lists overhead
        merged = pc2.create_cloud_xyz32(header, merged_pts.astype(np.float32))
        self.pub.publish(merged)


def main():
    rclpy.init()
    node = CloudMerger()
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
