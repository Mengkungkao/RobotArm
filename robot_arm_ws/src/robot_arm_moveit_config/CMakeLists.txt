cmake_minimum_required(VERSION 3.8)
project(robot_arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# MoveItConfigsBuilder looks for this file in the package share directory.
install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

# Re-audit the collision matrix after a geometry change; see the script header.
install(
  PROGRAMS scripts/collision_audit.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # Style gate for this package's Python.  ament_flake8 uses its own bundled
  # config (import-order-style = google), not any setup.cfg in the tree.
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py
    TIMEOUT 120 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
