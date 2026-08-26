// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
//
// Minimal application built on the C++ API.  It runs unchanged against
// Gazebo and against the physical robot:
//
//   ros2 launch robot_arm_bringup sim.launch.py
//   ros2 launch robot_arm_control cpp_api_demo.launch.py
//
//   ros2 launch robot_arm_bringup real.launch.py
//   ros2 launch robot_arm_control cpp_api_demo.launch.py

#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_arm_control/robot_arm_client.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MoveGroupInterface needs its node to be spinning, and needs the
  // robot_description parameters that the launch file passes in.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("cpp_api_demo", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  robot_arm_control::RobotArmClient robot(node);
  const auto logger = node->get_logger();

  if (const auto result = robot.enable(); !result) {
    RCLCPP_WARN(logger, "enable(): %s", result.message.c_str());
  }

  // 1. joint-space motion
  RCLCPP_INFO(logger, "Moving in joint space");
  auto result = robot.moveJoints({0.0, 0.5, -0.8, 0.0, 0.5, 0.0});
  RCLCPP_INFO(logger, "  -> %s", result.message.c_str());

  // 2. Cartesian motion
  RCLCPP_INFO(logger, "Moving to a Cartesian pose");
  result = robot.moveToPose(0.35, 0.10, 0.40, 0.0, 1.57, 0.0);
  RCLCPP_INFO(logger, "  -> %s", result.message.c_str());

  // 3. read the state back
  const auto states = robot.getJointStates();
  for (std::size_t i = 0; i < states.names.size(); ++i) {
    RCLCPP_INFO(
      logger, "  %s = %.4f rad", states.names[i].c_str(), states.positions[i]);
  }
  const auto pose = robot.getCurrentPose();
  RCLCPP_INFO(
    logger, "tool0 at [%.3f, %.3f, %.3f] in %s",
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
    pose.header.frame_id.c_str());

  robot.stop();

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
