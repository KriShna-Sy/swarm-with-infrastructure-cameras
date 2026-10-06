"""
rviz.launch.py
==============
Launches RViz2 pre-configured to display:
  - Robot model (TF)
  - Occupancy map (/map)
  - RTAB-Map point cloud (/rtabmap/cloud_map)
  - Camera feed (/infra/cam_oak/rgb/image)
  - Odometry path (/model/fwld/odometry)

Terminal 4 (optional, for visualisation):
    source ~/swarm2_ws/install/setup.bash
    ros2 launch swarm_description rviz.launch.py
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

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

    ])
