#!/usr/bin/env python3
"""
task_manager.py
================
Detects objects to be sorted from /merged_cloud and dispatches idle
robots to fetch them.

Objects vs. robots: both show up as "foreground" (not in the static
background) in the same clustering used by infra_localizer.py. We tell
them apart by behaviour, not appearance -- a robot is continuously near
one of the live /<robot>/infra_pose estimates; anything else that stays
in roughly the same place for `stable_seconds` is a candidate object.
This avoids needing a second detector/classifier.

Per object: once stable, if any robot is idle, assign the nearest idle
one by publishing PoseStamped on /<robot>/pick_goal. Robot status comes
from /<robot>/task_status (published by robot_task_executor.py).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def flood_fill_clusters(occ):
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
                    if 0 <= nr < rows and 0 <= nc < cols \
                            and occ[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        stack.append((nr, nc))
            clusters.append((cr_list, cc_list))
    return clusters


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        self.declare_parameter('robot_names', ['fwld_1', 'fwld_2'])
        self.declare_parameter('room_half_extent', 2.9)
        self.declare_parameter('cell_size', 0.05)
        self.declare_parameter('calib_seconds', 5.0)
        self.declare_parameter('stable_seconds', 2.0)
        self.declare_parameter('robot_exclusion_radius', 0.20)
        self.declare_parameter('object_min_cells', 2)
        self.declare_parameter('object_max_cells', 40)   # excludes robot-sized blobs

        self.robot_names = self.get_parameter('robot_names').value
        half = self.get_parameter('room_half_extent').value
        self.cell = self.get_parameter('cell_size').value
        self.n = int(2 * half / self.cell) + 1
        self.origin = -half
        self.calib_seconds = self.get_parameter('calib_seconds').value
        self.stable_seconds = self.get_parameter('stable_seconds').value
        self.excl_r = self.get_parameter('robot_exclusion_radius').value
        self.min_cells = self.get_parameter('object_min_cells').value
        self.max_cells = self.get_parameter('object_max_cells').value

        self.background = np.zeros((self.n, self.n), dtype=bool)
        self.start_time = None

        self.robot_pose = {name: None for name in self.robot_names}
        self.robot_status = {name: 'idle' for name in self.robot_names}
        self.pick_pubs = {}
        for name in self.robot_names:
            self.create_subscription(
                PoseStamped, f'/{name}/infra_pose',
                lambda msg, n=name: self.robot_pose.__setitem__(n, (msg.pose.position.x, msg.pose.position.y)),
                10)
            self.create_subscription(
                String, f'/{name}/task_status',
                lambda msg, n=name: self.robot_status.__setitem__(n, msg.data), 10)
            self.pick_pubs[name] = self.create_publisher(PoseStamped, f'/{name}/pick_goal', 10)

        # object_id -> {'xy': (x,y), 'first_seen': t, 'assigned': bool}
        self.objects = {}
        self.next_id = 0

        self.create_subscription(PointCloud2, '/merged_cloud', self.cb_cloud, 10)
        self.get_logger().info(
            f'task_manager started -- calibrating {self.calib_seconds}s')

    def _xy_to_cell(self, x, y):
        return int((y - self.origin) / self.cell), int((x - self.origin) / self.cell)

    def _cell_to_xy(self, r, c):
        return self.origin + (c + 0.5) * self.cell, self.origin + (r + 0.5) * self.cell

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

        t_sec = now.nanoseconds / 1e9
        for cr_list, cc_list in flood_fill_clusters(foreground):
            n_cells = len(cr_list)
            if not (self.min_cells <= n_cells <= self.max_cells):
                continue  # too small (noise) or too big (that's a robot)

            xs, ys = zip(*(self._cell_to_xy(r, c) for r, c in zip(cr_list, cc_list)))
            cx, cy = float(np.mean(xs)), float(np.mean(ys))

            # skip anything near a known robot position
            near_robot = any(
                pos is not None and (pos[0] - cx) ** 2 + (pos[1] - cy) ** 2 < self.excl_r ** 2
                for pos in self.robot_pose.values())
            if near_robot:
                continue

            self._update_object(cx, cy, t_sec)

        self._assign_idle_robots()

    def _update_object(self, x, y, t_sec):
        for obj in self.objects.values():
            ox, oy = obj['xy']
            if (ox - x) ** 2 + (oy - y) ** 2 < (2 * self.cell) ** 2:
                obj['xy'] = (x, y)
                return
        obj_id = self.next_id
        self.next_id += 1
        self.objects[obj_id] = {'xy': (x, y), 'first_seen': t_sec, 'assigned': False}
        self.get_logger().info(f'candidate object {obj_id} at ({x:.2f}, {y:.2f})')

    def _assign_idle_robots(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        ready = [
            (oid, o) for oid, o in self.objects.items()
            if not o['assigned'] and (now_sec - o['first_seen']) >= self.stable_seconds
        ]
        if not ready:
            return
        idle = [n for n in self.robot_names if self.robot_status.get(n) == 'idle']
        if not idle:
            return

        for oid, obj in ready:
            if not idle:
                break
            ox, oy = obj['xy']
            best = min(idle, key=lambda n: (
                (self.robot_pose[n][0] - ox) ** 2 + (self.robot_pose[n][1] - oy) ** 2
                if self.robot_pose[n] else float('inf')))
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = ox
            goal.pose.position.y = oy
            goal.pose.orientation.w = 1.0
            self.pick_pubs[best].publish(goal)
            obj['assigned'] = True
            idle.remove(best)
            self.get_logger().info(f'assigned object {oid} to {best}')


def main():
    rclpy.init()
    node = TaskManager()
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
