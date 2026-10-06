cmake_minimum_required(VERSION 3.8)
project(swarm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    launch
    worlds
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# Python nodes, installed as executables under lib/${PROJECT_NAME} so
# `ros2 run swarm_description cloud_merger.py` / `infra_localizer.py`
# work from any machine -- no more hardcoded /home/krishna/... paths.
install(
  PROGRAMS
    scripts/cloud_merger.py
    scripts/infra_localizer.py
    scripts/task_manager.py
    scripts/robot_task_executor.py
    scripts/led_marker_publisher.py
    scripts/map_server_node.py
    scripts/swarm_navigator.py
    scripts/csi_ekf_localizer.py
    scripts/csi_swarm_controller.py
    scripts/swarm_research_evaluator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
