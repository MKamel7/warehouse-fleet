cmake_minimum_required(VERSION 3.8)
project(warehouse_bot_package)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf launch config maps params rviz
  DESTINATION share/${PROJECT_NAME}
)

# Shared, ROS-free helper module imported by the nodes (sits alongside them in
# lib/ so a script's `import wb_common` resolves).
install(PROGRAMS
  scripts/wb_common.py
  DESTINATION lib/${PROJECT_NAME}
)

# Fleet / convoy / task nodes
install(PROGRAMS
  scripts/fleet_coordinator.py
  scripts/fleet_send_goals.py
  scripts/convoy_controller.py
  scripts/gt_localization.py
  scripts/leader_controller.py
  scripts/task_allocator.py
  scripts/submit_task.py
  scripts/convoy_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(wb_common_test test/test_wb_common.py)
  ament_add_pytest_test(traffic_test test/test_traffic.py)
endif()

ament_package()
