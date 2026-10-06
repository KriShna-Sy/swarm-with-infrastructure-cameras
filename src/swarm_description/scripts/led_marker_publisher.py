#!/usr/bin/env python3
"""
led_marker_publisher.py
========================
The chassis LED (${robot_name}_led link in fwld.urdf.xacro) is a static
Gazebo visual -- Gazebo Harmonic doesn't expose a simple runtime
material-recolor API, so we don't try to fight that for a demo. Instead
this publishes a colored visualization_msgs/Marker sphere at the LED's
TF frame, one per robot, driven by each robot's /<robot>/led_state
topic (std_msgs/String: "idle" | "busy" | "error"). Add a MarkerArray
display in RViz pointed at /led_markers.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

COLORS = {
    'idle':  (0.0, 1.0, 0.0),
    'busy':  (1.0, 0.8, 0.0),
    'error': (1.0, 0.0, 0.0),
}


class LedMarkerPublisher(Node):
    def __init__(self):
        super().__init__('led_marker_publisher')
        self.declare_parameter('robot_names', ['fwld_1', 'fwld_2'])
        self.robot_names = self.get_parameter('robot_names').value
        self.state = {name: 'idle' for name in self.robot_names}

        for name in self.robot_names:
            self.create_subscription(
                String, f'/{name}/led_state',
                lambda msg, n=name: self.state.__setitem__(n, msg.data), 10)

        self.pub = self.create_publisher(MarkerArray, '/led_markers', 10)
        self.create_timer(0.2, self.publish_markers)

    def publish_markers(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, name in enumerate(self.robot_names):
            r, g, b = COLORS.get(self.state[name], (0.5, 0.5, 0.5))
            m = Marker()
            m.header.frame_id = f'{name}_led'
            m.header.stamp = now
            m.ns = 'led'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.03
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.pose.orientation.w = 1.0
            m.lifetime.sec = 1
            arr.markers.append(m)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = LedMarkerPublisher()
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
