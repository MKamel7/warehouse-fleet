cmake_minimum_required(VERSION 3.8)
project(warehouse_bot_package)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf launch config maps params rviz
  DESTINATION share/${PROJECT_NAME}
)

# Inter-robot communication demo nodes
install(PROGRAMS
  scripts/fleet_coordinator.py
  scripts/fleet_send_goals.py
  scripts/convoy_controller.py
  scripts/gt_localization.py
  scripts/leader_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
