#!/usr/bin/env python3
"""
map_server_node.py
==================
Generates and broadcasts a 2D OccupancyGrid map for the confined_space room.
Accurately models the 6m x 6m room walls and all 6 obstacles defined in
confined_space.sdf:
  - obs_box_1 (0.5, -1.5)
  - obs_cyl_1 (-1.8, 0.5)
  - obs_box_2 (1.8, 1.8, yaw 45 deg)
  - obs_divider (0.0, 0.5)
  - obs_cyl_2 (2.0, -1.0)
  - obs_Lshape_a (-2.0, 2.0) & obs_Lshape_b (-1.75, 1.75)
Publishes on /map and /map_metadata for Nav2, RViz, and global path planners.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header


class MapServerNode(Node):
    def __init__(self):
        super().__init__('room_map_server')

        self.declare_parameter('resolution', 0.05)   # 5 cm per cell
        self.declare_parameter('room_size', 6.0)     # 6m x 6m
        self.declare_parameter('margin', 0.5)        # extra border margin
        self.declare_parameter('frame_id', 'map')

        self.res = float(self.get_parameter('resolution').value)
        self.room_size = float(self.get_parameter('room_size').value)
        self.margin = float(self.get_parameter('margin').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.total_size = self.room_size + 2 * self.margin
        self.width_cells = int(math.ceil(self.total_size / self.res))
        self.height_cells = self.width_cells
        self.origin_x = -self.total_size / 2.0
        self.origin_y = -self.total_size / 2.0

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.metadata_pub = self.create_publisher(MapMetaData, '/map_metadata', qos)

        self.grid = self._generate_occupancy_grid()
        self.timer = self.create_timer(10.0, self.publish_map)
        # Publish immediately on startup
        self._init_timer = self.create_timer(0.5, self._initial_publish)
        self.get_logger().info(f'Room map server initialized: {self.width_cells}x{self.height_cells} @ {self.res}m')

    def _initial_publish(self):
        self.publish_map()
        if self._init_timer is not None:
            self._init_timer.cancel()
            self._init_timer = None

    def _world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.res)
        row = int((y - self.origin_y) / self.res)
        return row, col

    def _fill_box(self, grid, cx, cy, sx, sy, yaw=0.0):
        c, s = math.cos(-yaw), math.sin(-yaw)
        r_half = math.hypot(sx, sy) / 2.0
        min_r, min_c = self._world_to_grid(cx - r_half, cy - r_half)
        max_r, max_c = self._world_to_grid(cx + r_half, cy + r_half)

        min_r = max(0, min_r)
        max_r = min(self.height_cells - 1, max_r)
        min_c = max(0, min_c)
        max_c = min(self.width_cells - 1, max_c)

        for r in range(min_r, max_r + 1):
            for col in range(min_c, max_c + 1):
                wx = self.origin_x + (col + 0.5) * self.res
                wy = self.origin_y + (r + 0.5) * self.res
                dx, dy = wx - cx, wy - cy
                rx = dx * c - dy * s
                ry = dx * s + dy * c
                if abs(rx) <= sx / 2.0 and abs(ry) <= sy / 2.0:
                    grid[r, col] = 100

    def _fill_cylinder(self, grid, cx, cy, radius):
        min_r, min_c = self._world_to_grid(cx - radius, cy - radius)
        max_r, max_c = self._world_to_grid(cx + radius, cy + radius)

        min_r = max(0, min_r)
        max_r = min(self.height_cells - 1, max_r)
        min_c = max(0, min_c)
        max_c = min(self.width_cells - 1, max_c)

        r2 = radius * radius
        for r in range(min_r, max_r + 1):
            for col in range(min_c, max_c + 1):
                wx = self.origin_x + (col + 0.5) * self.res
                wy = self.origin_y + (r + 0.5) * self.res
                if (wx - cx) ** 2 + (wy - cy) ** 2 <= r2:
                    grid[r, col] = 100

    def _generate_occupancy_grid(self):
        # 0 = free, 100 = occupied
        grid = np.zeros((self.height_cells, self.width_cells), dtype=np.int8)

        # Room boundary walls (6m x 6m centered at 0,0 => from -3.0 to +3.0)
        wall_thick = 0.15
        # South wall
        self._fill_box(grid, 0.0, -3.05, 6.2, wall_thick)
        # North wall
        self._fill_box(grid, 0.0, 3.05, 6.2, wall_thick)
        # West wall
        self._fill_box(grid, -3.05, 0.0, wall_thick, 6.2)
        # East wall
        self._fill_box(grid, 3.05, 0.0, wall_thick, 6.2)

        # Outside room perimeter marked as occupied
        for r in range(self.height_cells):
            for col in range(self.width_cells):
                wx = self.origin_x + (col + 0.5) * self.res
                wy = self.origin_y + (r + 0.5) * self.res
                if abs(wx) > 3.0 or abs(wy) > 3.0:
                    grid[r, col] = 100

        # Obstacles from confined_space.sdf:
        # obs_box_1: pose 0.5 -1.5, size 0.4 0.4
        self._fill_box(grid, 0.5, -1.5, 0.4, 0.4)

        # obs_cyl_1: pose -1.8 0.5, radius 0.2
        self._fill_cylinder(grid, -1.8, 0.5, 0.2)

        # obs_box_2: pose 1.8 1.8, size 0.5 0.3, yaw 0.785
        self._fill_box(grid, 1.8, 1.8, 0.5, 0.3, yaw=0.7854)

        # obs_divider: pose 0 0.5, size 1.2 0.1
        self._fill_box(grid, 0.0, 0.5, 1.2, 0.1)

        # obs_cyl_2: pose 2.0 -1.0, radius 0.15
        self._fill_cylinder(grid, 2.0, -1.0, 0.15)

        # obs_Lshape_a: pose -2.0 2.0, size 0.6 0.1
        self._fill_box(grid, -2.0, 2.0, 0.6, 0.1)

        # obs_Lshape_b: pose -1.75 1.75, size 0.6 0.1, yaw 1.5708
        self._fill_box(grid, -1.75, 1.75, 0.6, 0.1, yaw=1.5708)

        return grid

    def publish_map(self):
        now = self.get_clock().now().to_msg()

        msg = OccupancyGrid()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        msg.info.map_load_time = now
        msg.info.resolution = self.res
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.flatten().tolist()

        self.map_pub.publish(msg)
        self.metadata_pub.publish(msg.info)


def main():
    rclpy.init()
    node = MapServerNode()
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
