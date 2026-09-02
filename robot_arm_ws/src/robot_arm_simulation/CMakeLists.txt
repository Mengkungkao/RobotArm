cmake_minimum_required(VERSION 3.8)
project(robot_arm_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # Style gate for this package's Python.  ament_flake8 uses its own bundled
  # config (import-order-style = google), not any setup.cfg in the tree.
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  ament_add_pytest_test(test_simulation_assets test/test_simulation_assets.py
    TIMEOUT 60 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
