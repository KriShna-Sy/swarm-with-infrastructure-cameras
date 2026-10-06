"""
spawn_swarm.launch.py
======================
Spawns N FWLD robots into Gazebo, each with a unique robot_name so their
links/joints/frames/topics don't collide (see fwld.urdf.xacro).

Previously this file only did RSP + spawn -- no controllers were needed
(native gz-sim-diff-drive-system, confirmed by spawn.launch.py's own
docstring), but the URDF itself had every link/joint/topic hardcoded
identically for both robots, so a second robot could never actually
coexist with the first. That's fixed at the URDF level; this launch
file just needs to pass a distinct robot_name to each instance and make
sure each robot_state_publisher's tf/tf_static land back on the shared
/tf, /tf_static topics (ROS2 namespacing a node otherwise privatises
those too, which would fragment the TF tree).
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

ROBOTS = [
    # (robot_name, namespace, spawn_x, spawn_y)
    ('fwld_1', 'robot1', -1.0, -0.5),
    ('fwld_2', 'robot2', 1.0, 0.5),
]


def generate_launch_description():

    pkg = get_package_share_directory('swarm_description')
    xacro_file = os.path.join(pkg, 'urdf', 'fwld.urdf.xacro')

    actions = []

    for i, (robot_name, ns, x, y) in enumerate(ROBOTS):
        robot_description = ParameterValue(
            Command(['xacro ', xacro_file, ' robot_name:=', robot_name]),
            value_type=str)

        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name=f'rsp_{robot_name}', namespace=ns,
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            remappings=[
                ('tf', '/tf'), ('tf_static', '/tf_static'),
                # same fix as spawn.launch.py: RSP's default 'joint_states'
                # subscription (-> /robotN/joint_states under this namespace)
                # never matches gz-sim-joint-state-publisher-system's actual
                # model-scoped topic.
                ('joint_states', f'/model/{robot_name}/joint_states'),
            ],
            output='screen'))

        actions.append(TimerAction(period=2.0 + 1.5 * i, actions=[
            Node(package='ros_gz_sim', executable='create', name=f'spawn_{robot_name}',
                 arguments=['-name', robot_name, '-topic', f'/{ns}/robot_description',
                            '-x', str(x), '-y', str(y), '-z', '0.036'],
                 output='screen'),
        ]))

    return LaunchDescription(actions)
