cmake_minimum_required(VERSION 3.8)
project(robot_arm_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Structural checks on the launch files: fast, no robot involved.
  ament_add_pytest_test(test_launch_files test/test_launch_files.py
    TIMEOUT 120 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  # End-to-end run of the real-robot stack against the loopback bus: brings up
  # the actual hardware plugin, the controllers and the safety monitor, then
  # commands motion and trips the emergency stop.  No hardware, no simulator.
  add_launch_test(test/test_bringup_loopback.py
    TIMEOUT 240
  )
endif()

ament_package()
