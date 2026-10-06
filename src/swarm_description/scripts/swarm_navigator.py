#!/usr/bin/env python3
"""
swarm_navigator.py
==================
Full-fledged autonomous navigation node for FWLD swarm robots in confined_space.

Features:
  1. 2D Costmap with dynamic inflation around obstacles and other swarm robots.
  2. A* Global Path Planning with line-of-sight shortcut smoothing.
  3. Pure Pursuit Local Controller with dynamic velocity profiling and curvature limits.
  4. Real-time TF / odom pose tracking in 'map' coordinate frame.
  5. RViz 2D Goal Pose support: send goals to /{robot_name}/goal_pose.
  6. Visualizes planned global trajectory on /{robot_name}/plan (nav_msgs/Path).
  7. Publishes motion commands to /model/{robot_name}/cmd_vel.
"""
import math
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_msgs.msg import String, Header
import tf2_ros


class SwarmNavigator(Node):
    def __init__(self):
        super().__init__('swarm_navigator')

        self.declare_parameter('robot_name', 'fwld_1')
        self.declare_parameter('other_robot_name', 'fwld_2')
        self.declare_parameter('robot_radius', 0.22)        # robot bounding radius (m)
        self.declare_parameter('safety_margin', 0.08)       # additional inflation (m)
        self.declare_parameter('max_linear_speed', 0.35)    # m/s
        self.declare_parameter('max_angular_speed', 1.2)   # rad/s
        self.declare_parameter('lookahead_dist', 0.25)      # Pure pursuit lookahead (m)
        self.declare_parameter('goal_tolerance', 0.08)      # m
        self.declare_parameter('angle_tolerance', 0.20)     # rad

        self.name = str(self.get_parameter('robot_name').value)
        self.other_name = str(self.get_parameter('other_robot_name').value)
        self.r_footprint = float(self.get_parameter('robot_radius').value)
        self.margin = float(self.get_parameter('safety_margin').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)
        self.lookahead = float(self.get_parameter('lookahead_dist').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.angle_tol = float(self.get_parameter('angle_tolerance').value)

        self.inflation_radius = self.r_footprint + self.margin

        # Map state
        self.map_msg = None
        self.static_grid = None
        self.inflated_grid = None
        self.res = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.width = 0
        self.height = 0

        # Robot state
        self.curr_pose = None       # (x, y, yaw)
        self.other_pose = None      # (x, y)
        self.active_goal = None     # (gx, gy, gyaw)
        self.current_path = []      # list of (x, y) waypoints
        self.path_idx = 0
        self.state = 'IDLE'

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, f'/model/{self.name}/cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, f'/{self.name}/plan', 10)
        self.status_pub = self.create_publisher(String, f'/{self.name}/nav_status', 10)

        # Subscriptions
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, map_qos)
        self.create_subscription(PoseStamped, f'/{self.name}/goal_pose', self.cb_goal, 10)
        # Also listen to global /goal_pose if directed or default
        if self.name == 'fwld_1':
            self.create_subscription(PoseStamped, '/goal_pose', self.cb_goal, 10)

        self.create_subscription(Odometry, f'/model/{self.name}/odometry', self.cb_odom, 10)
        self.create_subscription(PoseStamped, f'/{self.name}/infra_pose', self.cb_infra_pose, 10)
        self.create_subscription(PoseStamped, f'/{self.other_name}/infra_pose', self.cb_other_infra, 10)

        # Main navigation control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f'SwarmNavigator started for robot: {self.name}')

    def cb_map(self, msg):
        raw_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        static = (raw_grid > 50) | (raw_grid < 0)
        if self.static_grid is not None and np.array_equal(self.static_grid, static):
            return
        self.map_msg = msg
        self.res = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.static_grid = static
        self.inflated_grid = self._inflate_grid(self.static_grid, self.inflation_radius)
        self.get_logger().info('Costmap received and inflated successfully')

    def cb_odom(self, msg):
        # Update orientation and speed estimate
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.curr_pose is not None:
            self.curr_pose = (self.curr_pose[0], self.curr_pose[1], yaw)

    def cb_infra_pose(self, msg):
        # High accuracy absolute map-frame position from infra localizer
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = self.curr_pose[2] if self.curr_pose is not None else 0.0
        self.curr_pose = (x, y, yaw)

    def cb_other_infra(self, msg):
        self.other_pose = (msg.pose.position.x, msg.pose.position.y)

    def cb_goal(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q = msg.pose.orientation
        gyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(f'[{self.name}] New goal received: ({gx:.2f}, {gy:.2f})')
        self.set_goal(gx, gy, gyaw)

    def set_goal(self, gx, gy, gyaw=0.0):
        self.active_goal = (gx, gy, gyaw)
        self._update_pose_from_tf()
        if self.curr_pose is None:
            self.get_logger().warn(f'[{self.name}] Robot pose unknown, cannot plan path!')
            self.state = 'BLOCKED'
            return False

        path = self.plan_path((self.curr_pose[0], self.curr_pose[1]), (gx, gy))
        if not path:
            self.get_logger().warn(f'[{self.name}] No collision-free path found to ({gx:.2f}, {gy:.2f})!')
            self.state = 'BLOCKED'
            self._publish_empty_path()
            return False

        self.current_path = path
        self.path_idx = 0
        self.state = 'NAVIGATING'
        self._publish_path(path)
        self.get_logger().info(f'[{self.name}] Planned path with {len(path)} waypoints')
        return True

    def cancel_goal(self):
        self.state = 'IDLE'
        self.active_goal = None
        self.current_path = []
        self.cmd_vel_pub.publish(Twist())
        self._publish_empty_path()

    def _update_pose_from_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', f'{self.name}_base_link', rclpy.time.Time(), timeout=Duration(seconds=0.05))
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.curr_pose = (tx, ty, yaw)
        except Exception:
            pass  # retain pose from infra localizer

    def _world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.res)
        row = int((y - self.origin_y) / self.res)
        return row, col

    def _grid_to_world(self, r, c):
        x = self.origin_x + (c + 0.5) * self.res
        y = self.origin_y + (r + 0.5) * self.res
        return x, y

    def _inflate_grid(self, grid, radius_m):
        rad_cells = int(math.ceil(radius_m / self.res))
        h, w = grid.shape
        inflated = np.copy(grid)
        occ_r, occ_c = np.where(grid)

        for dr in range(-rad_cells, rad_cells + 1):
            for dc in range(-rad_cells, rad_cells + 1):
                if dr * dr + dc * dc <= rad_cells * rad_cells:
                    nr = np.clip(occ_r + dr, 0, h - 1)
                    nc = np.clip(occ_c + dc, 0, w - 1)
                    inflated[nr, nc] = True
        return inflated

    def _is_collision_free(self, costmap, x0, y0, x1, y1):
        """Line-of-sight raycast check between two world points."""
        r0, c0 = self._world_to_grid(x0, y0)
        r1, c1 = self._world_to_grid(x1, y1)
        steps = max(abs(r1 - r0), abs(c1 - c0)) * 2
        if steps == 0:
            return not costmap[r0, c0]

        for s in range(steps + 1):
            r = int(round(r0 + (r1 - r0) * s / steps))
            c = int(round(c0 + (c1 - c0) * s / steps))
            if r < 0 or r >= self.height or c < 0 or c >= self.width:
                return False
            if costmap[r, c]:
                return False
        return True

    def plan_path(self, start_xy, goal_xy):
        if self.inflated_grid is None:
            return []

        costmap = np.copy(self.inflated_grid)
        # Dynamically inject other robot as obstacle
        if self.other_pose is not None:
            orow, ocol = self._world_to_grid(self.other_pose[0], self.other_pose[1])
            rad_cells = int(math.ceil((self.r_footprint * 2.0) / self.res))
            for dr in range(-rad_cells, rad_cells + 1):
                for dc in range(-rad_cells, rad_cells + 1):
                    if dr * dr + dc * dc <= rad_cells * rad_cells:
                        nr, nc = orow + dr, ocol + dc
                        if 0 <= nr < self.height and 0 <= nc < self.width:
                            costmap[nr, nc] = True

        sr, sc = self._world_to_grid(start_xy[0], start_xy[1])
        gr, gc = self._world_to_grid(goal_xy[0], goal_xy[1])

        # Clamp start & goal within map bounds
        sr = max(0, min(self.height - 1, sr))
        sc = max(0, min(self.width - 1, sc))
        gr = max(0, min(self.height - 1, gr))
        gc = max(0, min(self.width - 1, gc))

        # If goal is inside inflation, find nearest free cell
        if costmap[gr, gc]:
            found = False
            for d in range(1, 20):
                for dr in range(-d, d + 1):
                    for dc in range(-d, d + 1):
                        nr, nc = gr + dr, gc + dc
                        if 0 <= nr < self.height and 0 <= nc < self.width and not costmap[nr, nc]:
                            gr, gc = nr, nc
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                return []

        # A* search
        open_set = []
        heapq.heappush(open_set, (0.0, (sr, sc)))
        came_from = {}
        g_score = { (sr, sc): 0.0 }

        motions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        found = False
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == (gr, gc):
                found = True
                break

            cr, cc = current
            for dr, dc, weight in motions:
                nr, nc = cr + dr, cc + dc
                if 0 <= nr < self.height and 0 <= nc < self.width:
                    if costmap[nr, nc]:
                        continue
                    tentative_g = g_score[current] + weight
                    next_node = (nr, nc)
                    if tentative_g < g_score.get(next_node, float('inf')):
                        came_from[next_node] = current
                        g_score[next_node] = tentative_g
                        h = math.hypot(nr - gr, nc - gc)
                        heapq.heappush(open_set, (tentative_g + h, next_node))

        if not found:
            return []

        # Reconstruct path
        grid_path = [(gr, gc)]
        curr = (gr, gc)
        while curr in came_from:
            curr = came_from[curr]
            grid_path.append(curr)
        grid_path.reverse()

        # Convert to world coordinates
        raw_waypoints = [self._grid_to_world(r, c) for r, c in grid_path]
        raw_waypoints[0] = start_xy
        raw_waypoints[-1] = goal_xy

        # Path smoothing via line-of-sight shortcutting
        pruned = [raw_waypoints[0]]
        i = 0
        while i < len(raw_waypoints) - 1:
            best_j = i + 1
            for j in range(len(raw_waypoints) - 1, i, -1):
                if self._is_collision_free(costmap, raw_waypoints[i][0], raw_waypoints[i][1],
                                           raw_waypoints[j][0], raw_waypoints[j][1]):
                    best_j = j
                    break
            pruned.append(raw_waypoints[best_j])
            i = best_j

        return pruned

    def control_loop(self):
        self._update_pose_from_tf()
        self.status_pub.publish(String(data=self.state.lower()))

        if self.state != 'NAVIGATING':
            return

        if self.curr_pose is None or not self.current_path:
            self.cmd_vel_pub.publish(Twist())
            return

        rx, ry, ryaw = self.curr_pose
        gx, gy, gyaw = self.active_goal

        dist_to_final = math.hypot(gx - rx, gy - ry)
        if dist_to_final < self.goal_tol:
            # At goal position, align final orientation
            yaw_err = math.atan2(math.sin(gyaw - ryaw), math.cos(gyaw - ryaw))
            if abs(yaw_err) < self.angle_tol or gyaw == 0.0:
                self.cmd_vel_pub.publish(Twist())
                self.state = 'SUCCEEDED'
                self.get_logger().info(f'[{self.name}] Goal successfully reached!')
                self._publish_empty_path()
                return
            else:
                cmd = Twist()
                cmd.angular.z = max(-self.max_ang, min(self.max_ang, 1.5 * yaw_err))
                self.cmd_vel_pub.publish(cmd)
                return

        # Advance along waypoints
        while self.path_idx < len(self.current_path) - 1:
            wp_x, wp_y = self.current_path[self.path_idx]
            if math.hypot(wp_x - rx, wp_y - ry) < self.lookahead:
                self.path_idx += 1
            else:
                break

        target_x, target_y = self.current_path[self.path_idx]

        # Check for dynamic obstacle blockage ahead
        if self.other_pose is not None:
            dist_to_other = math.hypot(self.other_pose[0] - rx, self.other_pose[1] - ry)
            dist_target_other = math.hypot(self.other_pose[0] - target_x, self.other_pose[1] - target_y)
            if dist_to_other < self.r_footprint * 2.2 and dist_target_other < dist_to_other:
                # Stop and yield or re-plan
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(f'[{self.name}] Yielding to {self.other_name}', throttle_duration_sec=2.0)
                return

        dx = target_x - rx
        dy = target_y - ry
        target_angle = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_angle - ryaw), math.cos(target_angle - ryaw))

        cmd = Twist()
        if abs(yaw_err) > 0.8:
            # Turn in place when facing away from next waypoint
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 2.0 * yaw_err))
        else:
            # Smooth Pure pursuit motion
            speed_factor = max(0.2, math.cos(yaw_err))
            dist_factor = min(1.0, dist_to_final / 0.5)
            cmd.linear.x = self.max_lin * speed_factor * dist_factor
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 2.2 * yaw_err))

        self.cmd_vel_pub.publish(cmd)

    def _publish_path(self, waypoints):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in waypoints:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.plan_pub.publish(msg)

    def _publish_empty_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        self.plan_pub.publish(msg)


def main():
    rclpy.init()
    node = SwarmNavigator()
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
