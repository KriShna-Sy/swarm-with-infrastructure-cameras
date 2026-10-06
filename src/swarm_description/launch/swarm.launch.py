"""
swarm.launch.py
================
Full multi-robot bring-up in one command: Gazebo + infra cameras, both
FWLD robots, the TF-corrected cloud merger, and infra_localizer.py
(cluster-based robot pose estimation from the merged cloud -- this is
what actually implements the README's "Robot pose (localised by cloud)"
box for more than one robot).

RTAB-Map is intentionally NOT included here: a single shared
/diff_drive_controller-style odom-driven RTAB-Map instance can only
localise one moving frame at a time, so it doesn't extend to N robots.
Use rtabmap.launch.py separately for the single-robot mapping demo, or
run rtabmap here in mapping-only mode (subscribe_odom:false) if you
want a static /map built once from the room's static geometry -- ask if
you want that wired in too.

Terminal 1:
    ros2 launch swarm_description swarm.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('swarm_description')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'robot_names': 'fwld_1,fwld_2'}.items(),
    )

    spawn_swarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'spawn_swarm.launch.py')),
    )

    tf_statics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'infra_statics.launch.py')),
    )

    cloud_merger = TimerAction(period=4.0, actions=[
        Node(package='swarm_description', executable='cloud_merger.py',
             name='cloud_merger', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])

    map_server = TimerAction(period=2.0, actions=[
        Node(package='swarm_description', executable='map_server_node.py',
             name='room_map_server', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])

    infra_localizer = TimerAction(period=5.0, actions=[
        Node(package='swarm_description', executable='infra_localizer.py',
             name='infra_localizer', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'robot_names': ['fwld_1', 'fwld_2'],
                 'spawn_x': [-1.0, 1.0],
                 'spawn_y': [-0.5, 0.5],
             }]),
    ])

    navigator_fwld_1 = TimerAction(period=6.0, actions=[
        Node(package='swarm_description', executable='swarm_navigator.py',
             name='nav_fwld_1', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'robot_name': 'fwld_1',
                 'other_robot_name': 'fwld_2',
                 'robot_radius': 0.20,
                 'safety_margin': 0.08,
                 'max_linear_speed': 0.35,
                 'max_angular_speed': 1.2,
             }]),
    ])

    navigator_fwld_2 = TimerAction(period=6.0, actions=[
        Node(package='swarm_description', executable='swarm_navigator.py',
             name='nav_fwld_2', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'robot_name': 'fwld_2',
                 'other_robot_name': 'fwld_1',
                 'robot_radius': 0.20,
                 'safety_margin': 0.08,
                 'max_linear_speed': 0.35,
                 'max_angular_speed': 1.2,
             }]),
    ])

    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_AUTOMATIC_DISCOVERY_RANGE', value='LOCALHOST'),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1'),
        gazebo,
        spawn_swarm,
        tf_statics,
        map_server,
        cloud_merger,
        infra_localizer,
        navigator_fwld_1,
        navigator_fwld_2,
    ])
