// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_control/robot_arm_client.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "robot_arm_interfaces/srv/set_e_stop.hpp"
#include "robot_arm_interfaces/srv/set_motor_enable.hpp"

namespace robot_arm_control
{

using namespace std::chrono_literals;

namespace
{

/// Call a service and return the response, or nullptr on timeout.
template<typename ServiceT>
typename ServiceT::Response::SharedPtr call_service(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  typename ServiceT::Request::SharedPtr request, double timeout)
{
  auto client = node->create_client<ServiceT>(name);
  if (!client->wait_for_service(std::chrono::duration<double>(timeout))) {
    RCLCPP_WARN(node->get_logger(), "%s is not available", name.c_str());
    return nullptr;
  }
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
    RCLCPP_WARN(node->get_logger(), "%s did not answer in time", name.c_str());
    return nullptr;
  }
  return future.get();
}

}  // namespace

RobotArmClient::RobotArmClient(const rclcpp::Node::SharedPtr & node, Options options)
: node_(node), options_(std::move(options))
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_, options_.group_name);
  move_group_->setMaxVelocityScalingFactor(options_.velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(options_.acceleration_scaling);
  move_group_->setPlanningTime(options_.planning_time);
  move_group_->setPoseReferenceFrame(options_.base_frame);
  move_group_->setEndEffectorLink(options_.end_effector_frame);
  if (!options_.planner_id.empty()) {
    move_group_->setPlannerId(options_.planner_id);
  }

  joint_names_ = move_group_->getJointNames();

  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      joint_state_ = *message;
    });

  RCLCPP_INFO(
    node_->get_logger(), "RobotArmClient ready: group '%s', planning frame '%s', tip '%s'",
    options_.group_name.c_str(), move_group_->getPlanningFrame().c_str(),
    move_group_->getEndEffectorLink().c_str());
}

MoveResult RobotArmClient::planAndExecute(const std::string & description)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planning_result = move_group_->plan(plan);
  if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
    return MoveResult{false, description + ": planning failed",
      planning_result.val, 0.0};
  }

  const auto execution_result = move_group_->execute(plan);
  const bool ok = execution_result == moveit::core::MoveItErrorCode::SUCCESS;
  return MoveResult{
    ok, description + (ok ? " succeeded" : ": execution failed"),
    execution_result.val, plan.planning_time_};
}

MoveResult RobotArmClient::moveJoints(const std::vector<double> & positions)
{
  if (positions.size() != joint_names_.size()) {
    return MoveResult{false, "expected " + std::to_string(joint_names_.size()) +
      " joint values, got " + std::to_string(positions.size()), 0, 0.0};
  }
  move_group_->setStartStateToCurrentState();
  if (!move_group_->setJointValueTarget(positions)) {
    return MoveResult{false, "joint target is outside the joint limits", 0, 0.0};
  }
  return planAndExecute("joint-space motion");
}

MoveResult RobotArmClient::moveToPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  quaternion.normalize();

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return moveToPose(pose);
}

MoveResult RobotArmClient::moveToPose(const geometry_msgs::msg::Pose & pose)
{
  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(pose, options_.end_effector_frame);
  return planAndExecute("Cartesian motion");
}

MoveResult RobotArmClient::moveLinear(const geometry_msgs::msg::Pose & pose, double min_fraction)
{
  move_group_->setStartStateToCurrentState();

  const std::vector<geometry_msgs::msg::Pose> waypoints{pose};
  moveit_msgs::msg::RobotTrajectory trajectory;
  // 5 mm interpolation, jump threshold disabled: the planner's own IK
  // continuity check is what keeps the path smooth here.
  const double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
  if (fraction < min_fraction) {
    return MoveResult{
      false, "only " + std::to_string(fraction * 100.0) +
      "% of the straight-line path is reachable", 0, 0.0};
  }

  const auto result = move_group_->execute(trajectory);
  const bool ok = result == moveit::core::MoveItErrorCode::SUCCESS;
  return MoveResult{ok, ok ? "linear motion succeeded" : "linear motion failed", result.val, 0.0};
}

JointStates RobotArmClient::getJointStates() const
{
  JointStates states;
  states.names = joint_names_;

  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  for (const auto & name : joint_names_) {
    const auto it = std::find(joint_state_.name.begin(), joint_state_.name.end(), name);
    if (it == joint_state_.name.end()) {
      states.positions.push_back(std::numeric_limits<double>::quiet_NaN());
      states.velocities.push_back(std::numeric_limits<double>::quiet_NaN());
      states.efforts.push_back(std::numeric_limits<double>::quiet_NaN());
      continue;
    }
    const auto index = static_cast<std::size_t>(
      std::distance(joint_state_.name.begin(), it));
    states.positions.push_back(
      index < joint_state_.position.size() ? joint_state_.position[index] :
      std::numeric_limits<double>::quiet_NaN());
    states.velocities.push_back(
      index < joint_state_.velocity.size() ? joint_state_.velocity[index] :
      std::numeric_limits<double>::quiet_NaN());
    states.efforts.push_back(
      index < joint_state_.effort.size() ? joint_state_.effort[index] :
      std::numeric_limits<double>::quiet_NaN());
  }
  return states;
}

geometry_msgs::msg::PoseStamped RobotArmClient::getCurrentPose() const
{
  return move_group_->getCurrentPose(options_.end_effector_frame);
}

MoveResult RobotArmClient::stop()
{
  // Stop the local execution first, then ask the safety monitor to abort the
  // trajectory controller: the first is instantaneous, the second is what a
  // goal sent by somebody else obeys too.
  move_group_->stop();

  auto response = call_service<std_srvs::srv::Trigger>(
    node_, "/robot_arm/stop", std::make_shared<std_srvs::srv::Trigger::Request>(),
    options_.service_timeout);
  if (!response) {
    return MoveResult{true, "local execution stopped (/robot_arm/stop unavailable)", 0, 0.0};
  }
  return MoveResult{response->success, response->message, 0, 0.0};
}

MoveResult RobotArmClient::setMotorEnable(bool enable)
{
  auto request = std::make_shared<robot_arm_interfaces::srv::SetMotorEnable::Request>();
  request->enable = enable;
  auto response = call_service<robot_arm_interfaces::srv::SetMotorEnable>(
    node_, "/robot_arm/set_motor_enable", request, options_.service_timeout);
  if (!response) {
    return MoveResult{false, "/robot_arm/set_motor_enable is not available", 0, 0.0};
  }
  return MoveResult{response->success, response->message, 0, 0.0};
}

MoveResult RobotArmClient::enable()
{
  return setMotorEnable(true);
}

MoveResult RobotArmClient::disable()
{
  return setMotorEnable(false);
}

MoveResult RobotArmClient::setEStop(bool engage, const std::string & reason)
{
  auto request = std::make_shared<robot_arm_interfaces::srv::SetEStop::Request>();
  request->engage = engage;
  request->reason = reason;
  auto response = call_service<robot_arm_interfaces::srv::SetEStop>(
    node_, "/robot_arm/set_e_stop", request, options_.service_timeout);
  if (!response) {
    return MoveResult{false, "/robot_arm/set_e_stop is not available", 0, 0.0};
  }
  return MoveResult{response->success, response->message, 0, 0.0};
}

}  // namespace robot_arm_control
