"""
gazebo.launch.py
================
Launches Gazebo Harmonic with the confined_space world and bridges all
topics to ROS2 via ros_gz_bridge.

Motion architecture:
  - gz-sim-diff-drive-system (inside URDF) drives wheels natively in Gazebo
  - /model/<robot>/cmd_vel     bridged ROS2->Gazebo (Twist), per robot
  - /model/<robot>/odometry    bridged Gazebo->ROS2 (Odometry), per robot
  - /tf                        bridged Gazebo->ROS2 (TFMessage), shared
    (safe to share: each robot's frame_id/child_frame_id are uniquely
    prefixed by ${robot_name} in fwld.urdf.xacro, so there's no collision
    on the shared /tf topic itself)

Infrastructure cameras (fixed in room):
  - OAK-D Lite     -> /infra/cam_oak/{rgb,depth}/{image,camera_info,points}
  - RealSense D435 -> /infra/cam_rs/{rgb,depth}/{image,camera_info,points}

Pass robot_names as a comma-separated list to bridge more than one robot,
e.g.: ros2 launch swarm_description gazebo.launch.py robot_names:=fwld_1,fwld_2

NOTE: this deliberately does NOT set GZ_SIM_SYSTEM_PLUGIN_PATH /
GZ_SIM_RESOURCE_PATH / LD_LIBRARY_PATH. An earlier version of this file
hardcoded guessed vendor paths there, which broke Gazebo's ability to
find its own plugins (no GUI, `create` timing out waiting for world
names). Your shell environment (sourced via /opt/ros/jazzy/setup.bash)
already sets these correctly -- don't override them here.

Terminal 1:
    source ~/swarm2_ws/install/setup.bash
    ros2 launch swarm_description gazebo.launch.py
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('swarm_description')
    world = os.path.join(pkg_share, 'worlds', 'confined_space.sdf')

    robot_names = [n for n in
                   LaunchConfiguration('robot_names').perform(context).split(',')
                   if n]

    bridges = [
        # ── Simulation clock ─────────────────────────────────────────
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

        # ── TF from diff-drive plugin (shared: frame names are unique
        #    per robot thanks to the ${robot_name} prefix in the URDF)
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

        # ── OAK-D Lite infrastructure camera ─────────────────────────
        '/infra/cam_oak/rgb/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/infra/cam_oak/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/infra/cam_oak/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/infra/cam_oak/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/infra/cam_oak/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

        # ── Intel RealSense D435 infrastructure camera ────────────────
        '/infra/cam_rs/rgb/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/infra/cam_rs/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/infra/cam_rs/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/infra/cam_rs/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/infra/cam_rs/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
    ]

    # ── Per-robot bridges (cmd_vel, odometry, joint_states, gripper) ──
    for name in robot_names:
        bridges += [
            f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/model/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/model/{name}/gripper/cmd_lift@std_msgs/msg/Float64]gz.msgs.Double',
            f'/model/{name}/gripper/cmd_finger_left@std_msgs/msg/Float64]gz.msgs.Double',
            f'/model/{name}/gripper/cmd_finger_right@std_msgs/msg/Float64]gz.msgs.Double',
        ]

    return [
        ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + bridges,
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_AUTOMATIC_DISCOVERY_RANGE', value='LOCALHOST'),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1'),
        DeclareLaunchArgument('robot_names', default_value='fwld',
                               description='Comma-separated robot/model names to bridge'),
        OpaqueFunction(function=_launch_setup),
    ])
