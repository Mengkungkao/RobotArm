cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

# Headless preview tool: renders the model to a PNG with no RViz and no
# display, which is how a geometry change gets eyeballed in CI.
install(
  PROGRAMS
    scripts/urdf_preview.py
    scripts/compact_xacro.py
    scripts/compute_inertia.py
    scripts/solve_acceleration_limits.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # Style gate for this package's Python.  ament_flake8 uses its own bundled
  # config (import-order-style = google), not any setup.cfg in the tree.
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  # Expands the Xacro model in every backend mode and validates the resulting
  # URDF (tree structure, joint limits, TF chain, inertial sanity).
  ament_add_pytest_test(test_urdf test/test_urdf.py
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
