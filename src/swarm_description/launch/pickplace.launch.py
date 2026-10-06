"""
pickplace.launch.py
====================
Brings up the pick-and-place layer on top of swarm.launch.py:
  - task_manager.py       (object detection + robot dispatch)
  - robot_task_executor.py, one per robot
  - led_marker_publisher.py (RViz status LEDs)

Run alongside swarm.launch.py:
    ros2 launch swarm_description swarm.launch.py
    ros2 launch swarm_description pickplace.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_NAMES = ['fwld_1', 'fwld_2']
DROP_ZONE = (2.3, -2.3)   # shared floor-level drop zone, room corner clear of clutter


def generate_launch_description():
    actions = [
        Node(package='swarm_description', executable='task_manager.py',
             name='task_manager', output='screen',
             parameters=[{'robot_names': ROBOT_NAMES}]),

        Node(package='swarm_description', executable='led_marker_publisher.py',
             name='led_marker_publisher', output='screen',
             parameters=[{'robot_names': ROBOT_NAMES}]),
    ]

    for name in ROBOT_NAMES:
        actions.append(Node(
            package='swarm_description', executable='robot_task_executor.py',
            name=f'task_executor_{name}', output='screen',
            parameters=[{
                'robot_name': name,
                'drop_x': DROP_ZONE[0],
                'drop_y': DROP_ZONE[1],
            }]))

    return LaunchDescription(actions)
