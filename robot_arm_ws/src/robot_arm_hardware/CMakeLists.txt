cmake_minimum_required(VERSION 3.16)
project(robot_arm_hardware LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_arm_msgs REQUIRED)
find_package(robot_arm_interfaces REQUIRED)

set(THIS_PACKAGE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  diagnostic_msgs
  std_msgs
  robot_arm_msgs
  robot_arm_interfaces
)

# ---------------------------------------------------------------------------
# Core library: units, safety, transports and protocols.
#
# It deliberately has NO ROS dependency, which keeps the logic that will drive
# a physical machine unit-testable in isolation and reusable outside ROS.
# ---------------------------------------------------------------------------
add_library(robot_arm_hardware_core SHARED
  src/parameter_utils.cpp
  src/joint_config.cpp
  src/safety_checker.cpp
  src/transport/transport.cpp
  src/transport/loopback_transport.cpp
  src/transport/serial_transport.cpp
  src/transport/can_transport.cpp
  src/transport/tcp_transport.cpp
  src/protocol/motor_protocol.cpp
  src/protocol/loopback_protocol.cpp
  src/protocol/simple_ascii_protocol.cpp
)
target_include_directories(robot_arm_hardware_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_arm_hardware_core PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# The ros2_control plugin itself.
# ---------------------------------------------------------------------------
add_library(robot_arm_hardware SHARED
  src/robot_arm_hardware_interface.cpp
)
target_include_directories(robot_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_arm_hardware PUBLIC cxx_std_17)
target_link_libraries(robot_arm_hardware robot_arm_hardware_core)
ament_target_dependencies(robot_arm_hardware ${THIS_PACKAGE_DEPENDS})

# Tell pluginlib where to find the class description.
pluginlib_export_plugin_description_file(hardware_interface robot_arm_hardware.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  TARGETS robot_arm_hardware_core robot_arm_hardware
  EXPORT export_robot_arm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_joint_config test/test_joint_config.cpp)
  target_link_libraries(test_joint_config robot_arm_hardware_core)

  ament_add_gtest(test_safety_checker test/test_safety_checker.cpp)
  target_link_libraries(test_safety_checker robot_arm_hardware_core)

  ament_add_gtest(test_transport test/test_transport.cpp)
  target_link_libraries(test_transport robot_arm_hardware_core)

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol robot_arm_hardware_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(robot_arm_hardware_core robot_arm_hardware)
ament_export_targets(export_robot_arm_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})

ament_package()
