"""
spawn.launch.py
===============
Spawns the FWLD robot into Gazebo.

Motion is handled by gz-sim-diff-drive-system (Gazebo native).
No controllers.launch.py needed — just T1 gazebo + T2 spawn.

Spawn z=0.0:
  base_footprint is at ground level.
  base_footprint→base_link joint lifts base_link by 0.036 m (wheel radius).
  Wheels are at z=0 relative to base_link → touch ground correctly.

Terminal 2 (after Gazebo is fully open and room is visible):
    source ~/swarm2_ws/install/setup.bash
    ros2 launch swarm_description spawn.launch.py
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg        = get_package_share_directory('swarm_description')
    xacro_file = os.path.join(pkg, 'urdf', 'fwld.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([

        # ── Robot State Publisher ─────────────────────────────────────
        # Publishes /robot_description and static TF (chassis joints).
        # Dynamic TF (odom → base_link) comes from Gazebo diff-drive plugin.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            remappings=[
                # gz-sim-joint-state-publisher-system publishes joint_states
                # on the model-scoped topic /model/fwld/joint_states (its
                # <topic> element is relative). RSP's default subscription
                # ('joint_states' -> plain /joint_states) never sees it
                # without this remap -- wheels/gripper never got dynamic TF.
                ('joint_states', '/model/fwld/joint_states'),
            ],
            output='screen'
        ),

        # ── Spawn robot into Gazebo ───────────────────────────────────
        # z=0.0: base_footprint at floor level.
        # The base_footprint_joint (offset 0.036 m) lifts base_link,
        # placing wheel centres exactly at floor level so wheels touch ground.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_fwld',
                    arguments=[
                        '-name',  'fwld',
                        '-topic', '/robot_description',
                        '-x',     '0.0',
                        '-y',     '0.0',
                        '-z',     '0.0',    # base_footprint at floor
                        '-Y',     '0.0',
                    ],
                    output='screen'
                )
            ]
        ),

    ])
