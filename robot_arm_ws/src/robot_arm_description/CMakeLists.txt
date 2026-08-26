cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Expands the Xacro model in every backend mode and validates the resulting
  # URDF (tree structure, joint limits, TF chain, inertial sanity).
  ament_add_pytest_test(test_urdf test/test_urdf.py
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
