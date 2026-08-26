// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_CONTROL__ROBOT_ARM_CLIENT_HPP_
#define ROBOT_ARM_CONTROL__ROBOT_ARM_CLIENT_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_arm_control
{

/// Outcome of a motion request.
struct MoveResult
{
  bool success{false};
  std::string message;
  int error_code{0};
  double planning_time{0.0};

  explicit operator bool() const {return success;}
};

/// Joint state snapshot, ordered like RobotArmClient::jointNames().
struct JointStates
{
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
};

/// C++ control API for the 6-DOF arm - the counterpart of the Python
/// `RobotArm` class, for applications that need the lower latency of a
/// compiled client.
///
/// Like the Python API it talks to MoveIt and to the safety services, so the
/// same program drives Gazebo and the physical machine.
///
/// The node passed in must be spun by the caller (MoveGroupInterface needs a
/// running executor), and must have `robot_description` and
/// `robot_description_semantic` available - the launch files in
/// robot_arm_bringup pass both.
class RobotArmClient
{
public:
  struct Options
  {
    std::string group_name{"arm"};
    std::string base_frame{"base_link"};
    std::string end_effector_frame{"tool0"};
    double velocity_scaling{0.3};
    double acceleration_scaling{0.3};
    double planning_time{5.0};
    std::string planner_id;
    double service_timeout{5.0};
  };

  RobotArmClient(const rclcpp::Node::SharedPtr & node, Options options = Options());

  // -- motion -------------------------------------------------------------

  /// Plan and execute a joint-space motion.  One value per joint, in radians.
  MoveResult moveJoints(const std::vector<double> & positions);

  /// Plan and execute a Cartesian motion of `tool0`.
  MoveResult moveToPose(
    double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0);

  /// Same, with an explicit pose message.
  MoveResult moveToPose(const geometry_msgs::msg::Pose & pose);

  /// Straight-line Cartesian motion; fails when less than `min_fraction` of
  /// the path is reachable, so a partial move never happens silently.
  MoveResult moveLinear(const geometry_msgs::msg::Pose & pose, double min_fraction = 0.9);

  // -- state --------------------------------------------------------------

  /// Latest joint positions, velocities and efforts.
  JointStates getJointStates() const;

  /// Current pose of the end-effector frame in the planning frame.
  geometry_msgs::msg::PoseStamped getCurrentPose() const;

  const std::vector<std::string> & jointNames() const {return joint_names_;}

  // -- safety / power -----------------------------------------------------

  /// Stop now: abort the running trajectory and hold the current pose.
  MoveResult stop();

  /// Energise the drives (real robot) / activate the controller (simulation).
  MoveResult enable();
  MoveResult disable();

  /// Engage or release the latched emergency stop.
  MoveResult setEStop(bool engage, const std::string & reason = "cpp api");

private:
  MoveResult planAndExecute(const std::string & description);
  MoveResult setMotorEnable(bool enable);

  rclcpp::Node::SharedPtr node_;
  Options options_;
  std::vector<std::string> joint_names_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState joint_state_;
};

}  // namespace robot_arm_control

#endif  // ROBOT_ARM_CONTROL__ROBOT_ARM_CLIENT_HPP_
