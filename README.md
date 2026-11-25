# Infrastructure-Camera Swarm Localization & Navigation (CSI-Swarm)

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Camera-less robots, smart room. A swarm of lightweight differential-drive
robots carry **no cameras at all**. Fixed infrastructure depth cameras
(OAK-D Lite + RealSense D435, simulated in Gazebo) watch the room. Their
point clouds are fused in the `map` frame, and each robot is localized from
that fused cloud. Then each robot gets its own collision-free plan.

Built on ROS 2 Jazzy + Gazebo Harmonic at IIT Hyderabad, targeting IEEE APSCON 2026.

## Architecture

```
Fixed room cameras (OAK-D Lite + RealSense D435)
        │
        ▼
  ros_gz_bridge ──► /infra/cam_oak/depth/points, /infra/cam_rs/depth/points
                              │
                              ▼
            cloud_merger.py (TF-correct fusion) ──► /merged_cloud (map frame)
                              │
            ┌─────────────────┴──────────────────┐
            ▼                                    ▼
   RTAB-Map (single-robot demo)       infra_localizer.py (swarm)
                                      background subtraction + clustering
                                      → per-robot pose, no onboard sensors
                                                 │
                                                 ▼
                 csi_ekf_localizer.py ──► swarm_navigator.py / csi_swarm_controller.py
                 (odom + camera EKF)      (A* + Pure Pursuit, covariance-aware CBF safety)
```

## Features

- **TF-correct multi-camera cloud fusion.** Points are transformed into `map` via `tf2_sensor_msgs`, not just relabeled.
- **Infrastructure-only localization** for N robots, so no per-robot camera or LiDAR is needed.
- **CSI-EKF** (`csi_ekf_localizer.py`) fuses 50 Hz wheel odometry with 10 Hz infra-camera detections. It uses range-adaptive measurement covariance, Mahalanobis/chi-square data association so robots don't swap identities, and occlusion coasting.
- **CC-CBF cooperative controller** (`csi_swarm_controller.py`): chance-constrained control barrier functions widen inter-robot safety margins when the EKF covariance grows, for example during occlusions.
- **Swarm navigation**: 5 cm occupancy grid, A* global planner, dynamic inflation that treats other robots as obstacles, and a Pure Pursuit controller.
- **Multi-robot pick-and-place** task dispatcher + per-robot state machine.
- **Namespaced URDF**: every link, joint and topic is prefixed by `robot_name`, so robots can coexist.

## Quick start

```bash
mkdir -p ~/swarm2_ws/src && cd ~/swarm2_ws/src
git clone https://github.com/KriShna-Sy/swarm-with-infrastructure-cameras.git
cp -r swarm-with-infrastructure-cameras/src/swarm_description .
cd ~/swarm2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash

# Multi-robot swarm (Gazebo + 2 robots + infra cameras)
ros2 launch swarm_description swarm.launch.py

# Autonomous navigation + RViz
ros2 launch swarm_description nav.launch.py
ros2 topic pub --once /fwld_1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: -1.0}, orientation: {w: 1.0}}}"
```

The **full run guide** has single-robot RTAB-Map, verification steps, every
launch file and the changelog. It is in
[`src/swarm_description/README.md`](src/swarm_description/README.md).

## Repository layout

```
src/swarm_description/
├── urdf/       fwld.urdf.xacro (robot_name-parameterized diff-drive robot)
├── worlds/     confined_space.sdf (6 m × 6 m room, obstacles, 2 infra cameras)
├── scripts/    cloud_merger, infra_localizer, csi_ekf_localizer, swarm_navigator,
│               map_server_node, task_manager, robot_task_executor, ...
├── launch/     gazebo, spawn, swarm, spawn_swarm, rtabmap, nav, pickplace, ...
└── rviz/       swarm_nav.rviz
apscon_benchmark_results.csv   benchmark log schema (EKF vs. naive RMSE, clearance)
```

## License

MIT, see [LICENSE](LICENSE).
