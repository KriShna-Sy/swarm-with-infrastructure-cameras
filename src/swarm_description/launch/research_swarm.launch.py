"""
research_swarm.launch.py
========================
Research-grade Multi-Robot Bringup for IEEE APSCON 2026:
  - Gazebo Harmonic simulation with dual infrastructure RGB-D cameras
  - CSI-EKF Multi-Sensor Fusion Localizer with 3-Sigma Covariance Ellipses
  - Chance-Constrained Control Barrier Function (CC-CBF) Swarm Controllers
  - Real-time Quantitative Research Benchmark Evaluator (exporting to CSV)
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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

    map_server = TimerAction(period=2.0, actions=[
        Node(package='swarm_description', executable='map_server_node.py',
             name='room_map_server', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])

    cloud_merger = TimerAction(period=4.0, actions=[
        Node(package='swarm_description', executable='cloud_merger.py',
             name='cloud_merger', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])

    # ── Proposed CSI-EKF Multi-Sensor Fusion Localizer ────────────────────────
    csi_ekf_localizer = TimerAction(period=5.0, actions=[
        Node(package='swarm_description', executable='csi_ekf_localizer.py',
             name='csi_ekf_localizer', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'robot_names': ['fwld_1', 'fwld_2'],
                 'spawn_x': [-1.0, 1.0],
                 'spawn_y': [-0.5, 0.5],
                 'calib_seconds': 4.0,
                 'gate_threshold': 9.21,
             }]),
    ])

    # ── Baseline Naive Cluster Localizer (for Real-time Comparative Benchmark) ─
    baseline_localizer = TimerAction(period=5.5, actions=[
        Node(package='swarm_description', executable='infra_localizer.py',
             name='baseline_infra_localizer', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'robot_names': ['fwld_1', 'fwld_2'],
                 'spawn_x': [-1.0, 1.0],
                 'spawn_y': [-0.5, 0.5],
             }]),
    ])

    # ── Proposed Chance-Constrained CBF Swarm Controllers ──────────────────────
    csi_controller_fwld_1 = TimerAction(period=6.5, actions=[
        Node(package='swarm_description', executable='csi_swarm_controller.py',
             name='csi_ctrl_fwld_1', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'robot_name': 'fwld_1',
                 'other_robot_name': 'fwld_2',
                 'robot_radius': 0.20,
                 'confidence_beta': 3.0,
                 'max_linear_speed': 0.35,
                 'max_angular_speed': 1.2,
                 'cbf_gamma': 1.5,
             }]),
    ])

    csi_controller_fwld_2 = TimerAction(period=6.5, actions=[
        Node(package='swarm_description', executable='csi_swarm_controller.py',
             name='csi_ctrl_fwld_2', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'robot_name': 'fwld_2',
                 'other_robot_name': 'fwld_1',
                 'robot_radius': 0.20,
                 'confidence_beta': 3.0,
                 'max_linear_speed': 0.35,
                 'max_angular_speed': 1.2,
                 'cbf_gamma': 1.5,
             }]),
    ])

    # ── Real-time Research Performance Evaluator ──────────────────────────────
    research_evaluator = TimerAction(period=7.0, actions=[
        Node(package='swarm_description', executable='swarm_research_evaluator.py',
             name='swarm_research_evaluator', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'robot_names': ['fwld_1', 'fwld_2'],
                 'eval_period_s': 1.0,
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
        csi_ekf_localizer,
        baseline_localizer,
        csi_controller_fwld_1,
        csi_controller_fwld_2,
        research_evaluator,
    ])
