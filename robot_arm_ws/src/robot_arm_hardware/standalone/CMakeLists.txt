# ===========================================================================
#  Standalone build of the ROS-independent core of robot_arm_hardware.
#
#  The transport layer, the motor protocols, the encoder/calibration
#  conversions and the safety checker deliberately have no ROS dependency, so
#  they can be built and tested anywhere - a CI container, a developer laptop
#  with no ROS installed, or a cross-compilation host.
#
#      cmake -S standalone -B build-core -DCMAKE_BUILD_TYPE=Release
#      cmake --build build-core -j
#      ctest --test-dir build-core --output-on-failure
#
#  This is NOT how the package is built for ROS: `colcon build` uses the
#  ament_cmake CMakeLists.txt in the parent directory, which additionally
#  builds the ros2_control plugin.  Both compile the same core sources.
# ===========================================================================
cmake_minimum_required(VERSION 3.16)
project(robot_arm_hardware_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_library(robot_arm_hardware_core
  ${PACKAGE_ROOT}/src/parameter_utils.cpp
  ${PACKAGE_ROOT}/src/joint_config.cpp
  ${PACKAGE_ROOT}/src/safety_checker.cpp
  ${PACKAGE_ROOT}/src/transport/transport.cpp
  ${PACKAGE_ROOT}/src/transport/loopback_transport.cpp
  ${PACKAGE_ROOT}/src/transport/serial_transport.cpp
  ${PACKAGE_ROOT}/src/transport/can_transport.cpp
  ${PACKAGE_ROOT}/src/transport/tcp_transport.cpp
  ${PACKAGE_ROOT}/src/protocol/motor_protocol.cpp
  ${PACKAGE_ROOT}/src/protocol/loopback_protocol.cpp
  ${PACKAGE_ROOT}/src/protocol/simple_ascii_protocol.cpp
)
target_include_directories(robot_arm_hardware_core PUBLIC ${PACKAGE_ROOT}/include)
find_package(Threads REQUIRED)
target_link_libraries(robot_arm_hardware_core PUBLIC Threads::Threads)

# ---------------------------------------------------------------------------
# Tests - the same sources colcon runs through ament_cmake_gtest.
# ---------------------------------------------------------------------------
include(CTest)
find_package(GTest)

if(GTest_FOUND AND BUILD_TESTING)
  foreach(suite joint_config safety_checker transport protocol)
    add_executable(test_${suite} ${PACKAGE_ROOT}/test/test_${suite}.cpp)
    target_link_libraries(test_${suite}
      robot_arm_hardware_core GTest::gtest GTest::gtest_main)
    add_test(NAME test_${suite} COMMAND test_${suite})
  endforeach()
else()
  message(STATUS "GTest not found: building the core library only")
endif()
