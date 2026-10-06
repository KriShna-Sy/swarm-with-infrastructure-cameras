"""
nav.launch.py
=============
Dedicated navigation launch for the FWLD swarm:
  - Publishes 2D Room Occupancy Grid (/map)
  - Swarm A* Planner & Pure Pursuit Navigator for fwld_1 & fwld_2
  - Launches RViz2 preconfigured with Map, RobotModels, PointCloud, and Path visualizers
"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('swarm_description')
    rviz_config = os.path.join(pkg, 'rviz', 'swarm_nav.rviz')

    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_AUTOMATIC_DISCOVERY_RANGE', value='LOCALHOST'),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1'),
        # ── Map Server ────────────────────────────────────────────────────────
        Node(
            package='swarm_description',
            executable='map_server_node.py',
            name='room_map_server',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # ── Swarm Navigators ──────────────────────────────────────────────────
        Node(
            package='swarm_description',
            executable='swarm_navigator.py',
            name='nav_fwld_1',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_name': 'fwld_1',
                'other_robot_name': 'fwld_2',
                'robot_radius': 0.20,
                'safety_margin': 0.08,
                'max_linear_speed': 0.35,
                'max_angular_speed': 1.2,
            }]
        ),

        Node(
            package='swarm_description',
            executable='swarm_navigator.py',
            name='nav_fwld_2',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_name': 'fwld_2',
                'other_robot_name': 'fwld_1',
                'robot_radius': 0.20,
                'safety_margin': 0.08,
                'max_linear_speed': 0.35,
                'max_angular_speed': 1.2,
            }]
        ),

        # ── RViz2 Visualizer ──────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_nav',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
