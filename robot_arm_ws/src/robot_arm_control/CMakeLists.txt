cmake_minimum_required(VERSION 3.16)
project(robot_arm_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(robot_arm_msgs REQUIRED)
find_package(robot_arm_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Safety monitor - runs in simulation and on hardware, needs no MoveIt.
# ---------------------------------------------------------------------------
add_executable(safety_monitor src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  controller_manager_msgs
  robot_arm_msgs
  robot_arm_interfaces
)
install(TARGETS safety_monitor DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# C++ user API.
#
# Built only when MoveIt is available, so that a simulation-only or
# hardware-only workspace (where MoveIt may not be installed) still gets the
# controllers, the safety monitor and the Python API.
# ---------------------------------------------------------------------------
find_package(moveit_ros_planning_interface QUIET)
find_package(moveit_msgs QUIET)
find_package(tf2 QUIET)
find_package(tf2_ros QUIET)

if(moveit_ros_planning_interface_FOUND)
  add_library(robot_arm_client SHARED src/robot_arm_client.cpp)
  target_include_directories(robot_arm_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(robot_arm_client PUBLIC cxx_std_17)
  ament_target_dependencies(robot_arm_client
    rclcpp
    geometry_msgs
    sensor_msgs
    std_srvs
    moveit_ros_planning_interface
    moveit_msgs
    tf2
    tf2_ros
    robot_arm_interfaces
  )

  add_executable(cpp_api_demo examples/cpp_api_demo.cpp)
  target_link_libraries(cpp_api_demo robot_arm_client)
  ament_target_dependencies(cpp_api_demo rclcpp)

  install(DIRECTORY include/ DESTINATION include)
  install(
    TARGETS robot_arm_client
    EXPORT export_robot_arm_control
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS cpp_api_demo DESTINATION lib/${PROJECT_NAME})

  ament_export_include_directories(include)
  ament_export_libraries(robot_arm_client)
  ament_export_targets(export_robot_arm_control HAS_LIBRARY_TARGET)
else()
  message(STATUS
    "moveit_ros_planning_interface not found: skipping the C++ API "
    "(robot_arm_client, cpp_api_demo). Everything else still builds.")
endif()

# ---------------------------------------------------------------------------
# Python API
# ---------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
    examples/python_api_demo.py
    scripts/analyse_duty_cycle.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Data
# ---------------------------------------------------------------------------
install(DIRECTORY config launch examples DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config test/test_config.py
    TIMEOUT 60 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_python_api test/test_python_api.py
    TIMEOUT 120 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  # Kinematics, the Jacobian and the static load the drives have to hold.
  ament_add_pytest_test(test_kinematics test/test_kinematics.py
    TIMEOUT 180 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  # Motion profiles and what a real duty cycle costs the drives.
  ament_add_pytest_test(test_duty_cycle test/test_duty_cycle.py
    TIMEOUT 180 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_export_dependencies(rclcpp robot_arm_msgs robot_arm_interfaces)
ament_package()
