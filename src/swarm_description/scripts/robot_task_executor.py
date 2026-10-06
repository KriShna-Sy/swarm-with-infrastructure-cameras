#!/usr/bin/env python3
"""
robot_task_executor.py
=======================
One instance per robot (set the `robot_name` param). Simple proportional
go-to-goal controller + gripper state machine:

  IDLE -> [pick_goal received] -> DRIVE_TO_PICK -> LOWER -> CLOSE
        -> RAISE -> DRIVE_TO_DROP -> LOWER -> OPEN -> RAISE -> IDLE

Publishes /<robot>/task_status ("idle"/"busy") for task_manager.py and
/<robot>/led_state ("idle"/"busy"/"error") for led_marker_publisher.py.

Gripper convention (must match fwld.urdf.xacro):
  finger joints: 0.0 = CLOSED, 0.5 = OPEN
  lift joint:    0.0 = UP,    -0.032 = DOWN (floor)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64

FINGER_OPEN = 0.5
FINGER_CLOSED = 0.0
LIFT_UP = 0.0
LIFT_DOWN = -0.032

STATES = ['IDLE', 'DRIVE_TO_PICK', 'LOWER_PICK', 'CLOSE', 'RAISE_PICK',
          'DRIVE_TO_DROP', 'LOWER_DROP', 'OPEN', 'RAISE_DROP']


class RobotTaskExecutor(Node):
    def __init__(self):
        super().__init__('robot_task_executor')

        self.declare_parameter('robot_name', 'fwld_1')
        self.declare_parameter('drop_x', 2.3)
        self.declare_parameter('drop_y', -2.3)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('angle_tolerance', 0.15)
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('actuation_settle_s', 1.5)
        self.declare_parameter('goal_timeout_s', 60.0)

        self.name = self.get_parameter('robot_name').value
        self.drop = (self.get_parameter('drop_x').value, self.get_parameter('drop_y').value)
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.settle_s = self.get_parameter('actuation_settle_s').value
        self.timeout_s = self.get_parameter('goal_timeout_s').value

        self.state = 'IDLE'
        self.pose = None          # (x, y, yaw)
        self.pick_xy = None
        self.nav_status = 'idle'
        self.goal_sent = False
        self.state_entered_t = self.get_clock().now()

        self.create_subscription(Odometry, f'/model/{self.name}/odometry', self.cb_odom, 10)
        self.create_subscription(PoseStamped, f'/{self.name}/infra_pose', self.cb_infra_pose, 10)
        self.create_subscription(PoseStamped, f'/{self.name}/pick_goal', self.cb_goal, 10)
        self.create_subscription(String, f'/{self.name}/nav_status', self.cb_nav_status, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, f'/model/{self.name}/cmd_vel', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, f'/{self.name}/goal_pose', 10)
        self.lift_pub = self.create_publisher(Float64, f'/model/{self.name}/gripper/cmd_lift', 10)
        self.finger_l_pub = self.create_publisher(Float64, f'/model/{self.name}/gripper/cmd_finger_left', 10)
        self.finger_r_pub = self.create_publisher(Float64, f'/model/{self.name}/gripper/cmd_finger_right', 10)
        self.status_pub = self.create_publisher(String, f'/{self.name}/task_status', 10)
        self.led_pub = self.create_publisher(String, f'/{self.name}/led_state', 10)

        self._set_gripper(FINGER_OPEN, LIFT_UP)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'robot_task_executor started for {self.name}')

    def cb_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.pose is not None:
            self.pose = (self.pose[0], self.pose[1], yaw)

    def cb_infra_pose(self, msg):
        # Ground truth pose in map coordinates
        yaw = self.pose[2] if self.pose is not None else 0.0
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def cb_nav_status(self, msg):
        self.nav_status = msg.data.lower()

    def cb_goal(self, msg):
        if self.state != 'IDLE':
            self.get_logger().warn('busy, ignoring new pick_goal')
            return
        self.pick_xy = (msg.pose.position.x, msg.pose.position.y)
        self._transition('DRIVE_TO_PICK')

    def _set_gripper(self, finger, lift):
        f = Float64(data=finger)
        self.finger_l_pub.publish(f)
        self.finger_r_pub.publish(f)
        self.lift_pub.publish(Float64(data=lift))

    def _send_nav_goal(self, target_xy):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(target_xy[0])
        msg.pose.position.y = float(target_xy[1])
        msg.pose.orientation.w = 1.0
        self.nav_goal_pub.publish(msg)
        self.goal_sent = True
        self.nav_status = 'navigating'

    def _transition(self, new_state):
        self.get_logger().info(f'{self.state} -> {new_state}')
        self.state = new_state
        self.goal_sent = False
        self.state_entered_t = self.get_clock().now()

    def _elapsed(self):
        return (self.get_clock().now() - self.state_entered_t).nanoseconds / 1e9

    def _publish_status(self):
        self.status_pub.publish(String(data='idle' if self.state == 'IDLE' else 'busy'))

    def _drive_toward(self, target_xy):
        """Proportional go-to-goal fallback if navigator is inactive."""
        if self.pose is None:
            return False
        x, y, yaw = self.pose
        tx, ty = target_xy
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist < self.goal_tol:
            self.cmd_vel_pub.publish(Twist())
            return True

        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

        cmd = Twist()
        if abs(yaw_err) > self.angle_tol:
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 2.0 * yaw_err))
        else:
            cmd.linear.x = max(0.0, min(self.max_lin, 0.6 * dist))
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 1.0 * yaw_err))
        self.cmd_vel_pub.publish(cmd)
        return False

    def tick(self):
        self._publish_status()

        if self.state == 'IDLE':
            self.led_pub.publish(String(data='idle'))
            return

        self.led_pub.publish(String(data='busy'))

        if self._elapsed() > self.timeout_s:
            self.get_logger().error(f'{self.state} timed out')
            self.led_pub.publish(String(data='error'))
            self.cmd_vel_pub.publish(Twist())
            self._transition('IDLE')
            return

        if self.state == 'DRIVE_TO_PICK':
            if not self.goal_sent:
                self._send_nav_goal(self.pick_xy)

            # Check if autonomous navigator succeeded or use direct drive fallback
            if self.nav_status == 'succeeded' or self._drive_toward(self.pick_xy):
                self._transition('LOWER_PICK')
                self._set_gripper(FINGER_OPEN, LIFT_DOWN)

        elif self.state == 'LOWER_PICK':
            if self._elapsed() > self.settle_s:
                self._transition('CLOSE')
                self._set_gripper(FINGER_CLOSED, LIFT_DOWN)

        elif self.state == 'CLOSE':
            if self._elapsed() > self.settle_s:
                self._transition('RAISE_PICK')
                self._set_gripper(FINGER_CLOSED, LIFT_UP)

        elif self.state == 'RAISE_PICK':
            if self._elapsed() > self.settle_s:
                self._transition('DRIVE_TO_DROP')

        elif self.state == 'DRIVE_TO_DROP':
            if not self.goal_sent:
                self._send_nav_goal(self.drop)

            if self.nav_status == 'succeeded' or self._drive_toward(self.drop):
                self._transition('LOWER_DROP')
                self._set_gripper(FINGER_CLOSED, LIFT_DOWN)

        elif self.state == 'LOWER_DROP':
            if self._elapsed() > self.settle_s:
                self._transition('OPEN')
                self._set_gripper(FINGER_OPEN, LIFT_DOWN)

        elif self.state == 'OPEN':
            if self._elapsed() > self.settle_s:
                self._transition('RAISE_DROP')
                self._set_gripper(FINGER_OPEN, LIFT_UP)

        elif self.state == 'RAISE_DROP':
            if self._elapsed() > self.settle_s:
                self.pick_xy = None
                self._transition('IDLE')


def main():
    rclpy.init()
    node = RobotTaskExecutor()
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
