"""
rtabmap.launch.py
==================
Single-robot RTAB-Map pipeline. Consumes the TF-corrected /merged_cloud
(see scripts/cloud_merger.py) and the robot's own odometry.

Fixes vs. the previous version:
  - Removed the static 'map'->'odom' publisher. RTAB-Map (subscribe_odom:
    True) owns and broadcasts that transform itself; having a static
    publisher on the same edge caused the two to fight over it.
  - 'delete_db_on_start' so stale /tmp/rtabmap_swarm.db from a previous
    run doesn't get silently reused/mixed with the current one.
  - cloud_merger.py launched via package share path (installed by
    CMakeLists), not a hardcoded /home/krishna/... path.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('swarm_description')

    tf_statics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'infra_statics.launch.py')))
    # NOTE: no static map->odom here -- rtabmap publishes it itself.

    return LaunchDescription([

        tf_statics,

        # ── Cloud merger — both cameras, TF-corrected, → /merged_cloud ──
        Node(
            package='swarm_description',
            executable='cloud_merger.py',
            name='cloud_merger',
            output='screen',
        ),

        # ── RTAB-Map — consuming merged cloud ─────────────────────────
        TimerAction(period=3.0, actions=[
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                parameters=[{
                    'use_sim_time':             True,
                    'subscribe_scan_cloud':     True,
                    'subscribe_odom':           True,
                    'subscribe_depth':          False,
                    'subscribe_rgb':            False,
                    'subscribe_stereo':         False,
                    'subscribe_rgbd':           False,

                    'frame_id':                 'fwld_base_link',
                    'odom_frame_id':            'odom',
                    'map_frame_id':             'map',

                    'approx_sync':              True,
                    'approx_sync_max_interval': 0.5,
                    'sync_queue_size':          30,

                    'Grid/CellSize':            '0.05',
                    'Grid/3D':                  'false',
                    'Grid/Sensor':              '0',
                    'Grid/RangeMax':            '8.0',

                    'Rtabmap/DetectionRate':    '1',
                    'Kp/MaxFeatures':           '-1',
                    'Vis/MinInliers':           '5',
                    'Mem/STMSize':              '30',
                    'Icp/MaxTranslation':       '3.0',

                    'database_path':            '/tmp/rtabmap_swarm.db',
                    'delete_db_on_start':       True,
                }],
                remappings=[
                    ('scan_cloud', '/merged_cloud'),
                    ('odom',       '/model/fwld/odometry'),
                ]
            ),
        ]),

        TimerAction(period=5.0, actions=[
            Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                name='rtabmap_viz',
                output='screen',
                parameters=[{
                    'use_sim_time':         True,
                    'subscribe_scan_cloud': True,
                    'subscribe_odom':       True,
                    'frame_id':             'fwld_base_link',
                    'approx_sync':          True,
                }],
                remappings=[
                    ('scan_cloud', '/merged_cloud'),
                    ('odom',       '/model/fwld/odometry'),
                ]
            ),
        ]),
    ])
