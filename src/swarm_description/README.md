# swarm_description — Complete Run Guide
## Infrastructure-Visual SLAM for Lightweight Swarms
### ROS2 Jazzy + Gazebo Harmonic | IIT Hyderabad

---

## What changed in this rework (APSCON pass)

1. **`cloud_merger.py` now actually transforms points into `map`** via
   `tf2_sensor_msgs.do_transform_cloud`. The previous version just
   overwrote `header.frame_id`, fusing two differently-posed cameras as
   if they were coincident -- this was the main source of garbage
   geometry in RTAB-Map.
2. **Fixed the TF conflict**: a static `map->odom` publisher used to run
   alongside RTAB-Map's own `map->odom` broadcast. Removed the static
   one; RTAB-Map owns that edge.
3. **`frame_id` was set to `'map'`** (i.e. equal to `map_frame_id`) in
   the RTAB-Map config, which is invalid -- it's now `'fwld_base_link'`,
   the robot's actual moving frame.
4. **No more hardcoded `/home/krishna/...` path.** `cloud_merger.py` and
   `infra_localizer.py` are installed as proper package executables
   (`ros2 run swarm_description cloud_merger.py`).
5. **Multi-robot spawn actually works now.** The URDF hardcoded every
   link/joint name and Gazebo topic (`base_link`, `r1`, `/diff_drive_controller/...`),
   so a second robot could never coexist with the first. Every name is
   now prefixed by `robot_name` (`fwld_1_base_link`, `/model/fwld_1/cmd_vel`, ...).
6. **New: `infra_localizer.py`** -- background-subtraction + clustering
   on `/merged_cloud` to estimate each robot's position from the infra
   cameras alone. This is what the architecture diagram below always
   claimed ("Robot pose localised by cloud") but nothing in the repo
   implemented -- a single odom-driven RTAB-Map instance can't localise
   more than one robot.
7. Removed `urdf/ros2_control.xacro` and `config/controllers.yaml` --
   dead config left over from before the switch to the native
   `gz-sim-diff-drive-system` plugin (confirmed unused by the URDF's own
   comments). Removed the duplicate/drifted `cloud_merger` copies and
   the stale `rtabmap.launchold.py`.
8. `delete_db_on_start: true` on the RTAB-Map node so a previous run's
   `/tmp/rtabmap_swarm.db` can't silently bleed into the current one.

---

## Architecture

```
Fixed Room Cameras (OAK-D Lite + RealSense D435)
        │
        ▼
  ros_gz_bridge  ──►  /infra/cam_oak/depth/points
                       /infra/cam_rs/depth/points
                              │
                              ▼
                cloud_merger.py (TF-correct) ──► /merged_cloud (map frame)
                              │
              ┌───────────────┴───────────────┐
              ▼                                 ▼
   RTAB-Map (single robot demo)      infra_localizer.py (swarm)
   builds /map, needs odom of          background-subtract + cluster
   the ONE robot being tracked         → per-robot pose, no odom needed
```

Robots carry **NO cameras** — they provide only wheel odometry (used by
the single-robot RTAB-Map demo). The multi-robot swarm case is
localised entirely from the infra cameras instead, since odom alone
can't disambiguate which robot owns which map update.

---

## One-Time Setup (Fresh Workspace)

```bash
mkdir -p ~/swarm2_ws/src
cd ~/swarm2_ws/src
cp -r /path/to/swarm_description .

sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rtabmap-ros ros-jazzy-rtabmap-viz \
  python3-numpy

cd ~/swarm2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Verify URDF parses
ros2 run xacro xacro src/swarm_description/urdf/fwld.urdf.xacro > /tmp/fwld.urdf
echo $?   # must print 0
```

---

## Single Robot + RTAB-Map (3 terminals)

```bash
# T1
ros2 launch swarm_description gazebo.launch.py
# T2 (after Gazebo/room is visible)
ros2 launch swarm_description spawn.launch.py
# T3
ros2 launch swarm_description rtabmap.launch.py
```

Drive it:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/model/fwld/cmd_vel
```
Keys: i=forward  ,=backward  j=left  l=right  k=stop

RViz: Map topic `/map`, PointCloud2 topic `/rtabmap/cloud_map`, TF,
RobotModel description topic `/robot_description`.

---

## Multi-Robot Swarm (1 terminal)

```bash
ros2 launch swarm_description swarm.launch.py
```

This brings up Gazebo (both cameras bridged), both robots (`fwld_1`,
`fwld_2`), the corrected cloud merger, and `infra_localizer.py`.

```bash
# drive each robot independently
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/model/fwld_1/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/model/fwld_2/cmd_vel

# watch the infra-camera pose estimates
ros2 topic echo /fwld_1/infra_pose
ros2 topic echo /fwld_2/infra_pose
```

`infra_localizer.py` needs ~5s at startup with the room clear of robots
to calibrate its static background (tune via the `calib_seconds` param
if your demo can't spare that).

---

## Verifying

```bash
ros2 topic list | grep infra          # infra camera topics present
ros2 topic list | grep oak            # nothing FROM the robot itself
ros2 topic list | grep 'model/fwld'   # per-robot cmd_vel/odometry
ros2 run tf2_tools view_frames        # confirm fwld_1_* / fwld_2_* frames don't collide
```

---

## Full-Fledged Autonomous Swarm Navigation

The swarm is equipped with collision-free autonomous navigation:
- **Global 2D Room Occupancy Map** (`/map`): 5cm resolution map accurately representing the 6m×6m room and all 6 obstacles.
- **A* Global Planner**: Plans collision-free paths around all walls and obstacles with line-of-sight smoothing.
- **Dynamic Costmap Inflation**: Inflates obstacle boundaries and dynamically treats other robots as moving obstacles.
- **Pure Pursuit Controller**: Tracks paths with smooth acceleration, curvature slowing, and dynamic obstacle yielding.

### Sending Navigation Goals:
1. **From RViz2**:
   Use the **2D Goal Pose** tool in the RViz top toolbar and click anywhere on `/map`.
2. **From Terminal (ROS 2 CLI)**:
   ```bash
   # Send fwld_1 to target (x=1.5, y=-1.0):
   ros2 topic pub --once /fwld_1/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}"

   # Send fwld_2 to target (x=-1.5, y=1.0):
   ros2 topic pub --once /fwld_2/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -1.5, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
   ```

### Launching Navigation Visualizer:
```bash
ros2 launch swarm_description nav.launch.py
```

---

## File Structure
```
swarm_description/
├── urdf/
│   └── fwld.urdf.xacro          ← robot URDF, robot_name-parameterized
├── worlds/
│   └── confined_space.sdf       ← 6m×6m room + obstacles + 2 infra cameras
├── scripts/
│   ├── cloud_merger.py          ← TF-correct multi-camera cloud fusion
│   ├── infra_localizer.py       ← per-robot pose from the merged cloud
│   ├── map_server_node.py       ← 2D room occupancy grid (/map)
│   ├── swarm_navigator.py       ← A* path planning & Pure Pursuit controller
│   ├── task_manager.py          ← multi-robot pick-and-place dispatcher
│   ├── robot_task_executor.py   ← pick-and-place state machine with navigation
│   └── led_marker_publisher.py  ← RViz status LEDs
├── launch/
│   ├── gazebo.launch.py         ← Gazebo + camera + per-robot bridges
│   ├── spawn.launch.py          ← single-robot RSP + spawn
│   ├── spawn_swarm.launch.py    ← multi-robot RSP + spawn
│   ├── infra_statics.launch.py  ← shared world/map + camera + odom TF tree
│   ├── swarm.launch.py          ← full swarm bring-up + map + navigators
│   ├── nav.launch.py            ← standalone navigation + RViz view
│   ├── pickplace.launch.py      ← autonomous pick-and-place mission
│   └── rviz.launch.py
├── rviz/
│   └── swarm_nav.rviz           ← pre-configured RViz visualization
└── README.md
```
