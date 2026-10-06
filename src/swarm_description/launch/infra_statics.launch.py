"""
infra_statics.launch.py
========================
Static TF tree for the room: world -> map, and both infra cameras'
extrinsics (must match worlds/confined_space.sdf model poses exactly --
they currently do: cam_oak at (-2.8,-2.8,2.0) pitch 30 deg yaw 45 deg,
cam_rs at (2.8,2.8,2.0) pitch 30 deg yaw 225 deg).

Factored out of rtabmap.launch.py so swarm.launch.py can reuse it
without duplicating five static_transform_publisher nodes.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_world_map',
             arguments=['--x', '0', '--y', '0', '--z', '0',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'world', '--child-frame-id', 'map'],
             parameters=[{'use_sim_time': True}]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_infra_cam_oak',
             arguments=['--x', '-2.8', '--y', '-2.8', '--z', '2.0',
                        '--roll', '0.0', '--pitch', '0.5236', '--yaw', '0.7854',
                        '--frame-id', 'world', '--child-frame-id', 'infra_cam_oak'],
             parameters=[{'use_sim_time': True}]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_cam_oak_rgbd',
             arguments=['--x', '0', '--y', '0', '--z', '0',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'infra_cam_oak',
                        '--child-frame-id', 'infra_cam_oak/link/cam_oak_rgbd'],
             parameters=[{'use_sim_time': True}]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_infra_cam_rs',
             arguments=['--x', '2.8', '--y', '2.8', '--z', '2.0',
                        '--roll', '0.0', '--pitch', '0.5236', '--yaw', '3.9270',
                        '--frame-id', 'world', '--child-frame-id', 'infra_cam_rs'],
             parameters=[{'use_sim_time': True}]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_cam_rs_rgbd',
             arguments=['--x', '0', '--y', '0', '--z', '0',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'infra_cam_rs',
                        '--child-frame-id', 'infra_cam_rs/link/cam_rs_rgbd'],
             parameters=[{'use_sim_time': True}]),

        # ── Map to robot odom origins (matches spawn positions in spawn_swarm.launch.py) ──
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_map_fwld_1_odom',
             arguments=['--x', '-1.0', '--y', '-0.5', '--z', '0',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'map', '--child-frame-id', 'fwld_1_odom'],
             parameters=[{'use_sim_time': True}]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_map_fwld_2_odom',
             arguments=['--x', '1.0', '--y', '0.5', '--z', '0',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'map', '--child-frame-id', 'fwld_2_odom'],
             parameters=[{'use_sim_time': True}]),
    ])
