cmake_minimum_required(VERSION 3.8)
project(robot_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_arm_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetMotorEnable.srv
  srv/SetEStop.srv
  srv/CalibrateJoint.srv
  srv/SaveCalibration.srv
  srv/GetCalibration.srv
  DEPENDENCIES robot_arm_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
