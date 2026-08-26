// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
//
// Safety monitor - the part of the safety system that must exist in BOTH
// modes.
//
// On the real robot the hardware interface already clamps every command inside
// the control loop.  Gazebo has no such layer, and an application that is only
// ever tested against the simulator would then meet the e-stop for the first
// time on the machine.  This node closes that gap: it watches /joint_states,
// owns the latched emergency stop, aborts trajectory execution and publishes
// diagnostics - identically in simulation and on hardware.
//
// It also provides the three services the user APIs call, so that
// enable/disable/stop mean something in every mode:
//
//   /robot_arm/set_motor_enable  -> the drives (real) or the controller (sim)
//   /robot_arm/set_e_stop        -> latched stop, forwarded to the driver
//   /robot_arm/stop              -> abort the trajectory and hold the pose

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "robot_arm_interfaces/srv/set_e_stop.hpp"
#include "robot_arm_interfaces/srv/set_motor_enable.hpp"
#include "robot_arm_msgs/msg/safety_status.hpp"

namespace robot_arm_control
{

using namespace std::chrono_literals;

struct JointLimits
{
  double min_position{-M_PI};
  double max_position{M_PI};
  double max_velocity{M_PI};
  double max_effort{100.0};
};

class SafetyMonitor : public rclcpp::Node
{
public:
  SafetyMonitor()
  : rclcpp::Node("safety_monitor")
  {
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joints", std::vector<std::string>{
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    controller_manager_ = declare_parameter<std::string>("controller_manager",
      "/controller_manager");
    arm_controller_ = declare_parameter<std::string>("arm_controller", "arm_controller");
    hardware_node_ = declare_parameter<std::string>("hardware_node", "/robot_arm_hardware");
    check_rate_ = declare_parameter<double>("check_rate", 20.0);
    joint_state_timeout_ = declare_parameter<double>("joint_state_timeout", 0.5);
    diagnostics_period_ = declare_parameter<double>("diagnostics_period", 1.0);
    velocity_scale_ = declare_parameter<double>("velocity_scale", 1.0);
    warn_margin_ = declare_parameter<double>("warn_margin", 0.05);
    stop_controller_on_estop_ = declare_parameter<bool>("stop_controller_on_estop", true);
    estop_on_violation_ = declare_parameter<bool>("estop_on_violation", true);
    violation_cycles_ = declare_parameter<int>("violation_tolerance_cycles", 3);

    for (const auto & joint : joint_names_) {
      JointLimits limits;
      limits.min_position =
        declare_parameter<double>("limits." + joint + ".min_position", limits.min_position);
      limits.max_position =
        declare_parameter<double>("limits." + joint + ".max_position", limits.max_position);
      limits.max_velocity =
        declare_parameter<double>("limits." + joint + ".max_velocity", limits.max_velocity);
      limits.max_effort =
        declare_parameter<double>("limits." + joint + ".max_effort", limits.max_effort);
      if (limits.min_position >= limits.max_position) {
        RCLCPP_ERROR(
          get_logger(), "%s: min_position >= max_position, the joint is unusable",
          joint.c_str());
      }
      limits_[joint] = limits;
    }

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        last_joint_state_ = *message;
        last_joint_state_time_ = now();
      }, options);

    e_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/e_stop", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (message->data && !e_stop_active_) {
          engage_e_stop("/e_stop topic");
        } else if (!message->data && e_stop_active_) {
          e_stop_active_ = false;
          RCLCPP_WARN(get_logger(), "Emergency stop released");
        }
      }, options);

    e_stop_publisher_ = create_publisher<std_msgs::msg::Bool>("/e_stop", rclcpp::QoS(10));
    safety_publisher_ = create_publisher<robot_arm_msgs::msg::SafetyStatus>(
      "/robot_arm/safety_status", rclcpp::QoS(10));
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      controller_manager_ + "/switch_controller", rmw_qos_profile_services_default,
      callback_group_);
    hardware_enable_client_ = create_client<robot_arm_interfaces::srv::SetMotorEnable>(
      hardware_node_ + "/set_motor_enable", rmw_qos_profile_services_default, callback_group_);
    hardware_e_stop_client_ = create_client<robot_arm_interfaces::srv::SetEStop>(
      hardware_node_ + "/set_e_stop", rmw_qos_profile_services_default, callback_group_);

    e_stop_service_ = create_service<robot_arm_interfaces::srv::SetEStop>(
      "/robot_arm/set_e_stop",
      std::bind(&SafetyMonitor::handle_set_e_stop, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
    enable_service_ = create_service<robot_arm_interfaces::srv::SetMotorEnable>(
      "/robot_arm/set_motor_enable",
      std::bind(&SafetyMonitor::handle_set_motor_enable, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/robot_arm/stop",
      std::bind(&SafetyMonitor::handle_stop, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);

    check_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, check_rate_)),
      std::bind(&SafetyMonitor::check, this), callback_group_);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.1, diagnostics_period_)),
      std::bind(&SafetyMonitor::publish_diagnostics, this), callback_group_);

    RCLCPP_INFO(
      get_logger(), "Safety monitor watching %zu joints at %.1f Hz",
      joint_names_.size(), check_rate_);
  }

private:
  // -- limit checking -----------------------------------------------------

  void check()
  {
    robot_arm_msgs::msg::SafetyStatus status;
    status.header.stamp = now();
    status.header.frame_id = "base_link";
    status.e_stop_active = e_stop_active_;
    status.level = e_stop_active_ ?
      robot_arm_msgs::msg::SafetyStatus::LEVEL_ESTOP :
      robot_arm_msgs::msg::SafetyStatus::LEVEL_OK;

    if (last_joint_state_time_.nanoseconds() == 0) {
      status.message = "waiting for the first /joint_states message";
      status.level = std::max<uint8_t>(
        status.level, robot_arm_msgs::msg::SafetyStatus::LEVEL_WARN);
      latest_status_ = status;
      safety_publisher_->publish(status);
      return;
    }

    const double age = (now() - last_joint_state_time_).seconds();
    if (age > joint_state_timeout_) {
      status.communication_timeout = true;
      status.level = robot_arm_msgs::msg::SafetyStatus::LEVEL_VIOLATION;
      status.message = "no /joint_states for " + std::to_string(age) + " s";
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s", status.message.c_str());
      latest_status_ = status;
      safety_publisher_->publish(status);
      return;
    }

    const auto state = last_joint_state_;
    bool warn = false;
    for (std::size_t i = 0; i < state.name.size(); ++i) {
      const auto entry = limits_.find(state.name[i]);
      if (entry == limits_.end()) {
        continue;   // a joint we do not supervise, e.g. a gripper
      }
      const auto & limits = entry->second;

      if (i < state.position.size()) {
        const double position = state.position[i];
        if (!std::isfinite(position)) {
          status.invalid_feedback = true;
          status.violating_joints.push_back(state.name[i]);
        } else if (position < limits.min_position || position > limits.max_position) {
          status.position_limit_violation = true;
          status.violating_joints.push_back(state.name[i]);
        } else if (position < limits.min_position + warn_margin_ ||
          position > limits.max_position - warn_margin_)
        {
          warn = true;
        }
      }

      if (i < state.velocity.size() && std::isfinite(state.velocity[i])) {
        if (std::abs(state.velocity[i]) > limits.max_velocity * velocity_scale_) {
          status.velocity_limit_violation = true;
          status.violating_joints.push_back(state.name[i]);
        }
      }

      if (i < state.effort.size() && std::isfinite(state.effort[i]) && limits.max_effort > 0.0) {
        if (std::abs(state.effort[i]) > limits.max_effort) {
          status.effort_limit_violation = true;
          status.violating_joints.push_back(state.name[i]);
        }
      }
    }

    const bool violation = status.position_limit_violation ||
      status.velocity_limit_violation || status.effort_limit_violation ||
      status.invalid_feedback;

    if (violation) {
      ++consecutive_violations_;
      status.level = robot_arm_msgs::msg::SafetyStatus::LEVEL_VIOLATION;
      status.message = "joint limit exceeded";
      // A single noisy sample is not a reason to stop a moving arm; a
      // persistent one is.
      if (estop_on_violation_ && consecutive_violations_ >= violation_cycles_ &&
        !e_stop_active_)
      {
        engage_e_stop("joint limit exceeded");
      }
    } else {
      consecutive_violations_ = 0;
      if (warn && status.level == robot_arm_msgs::msg::SafetyStatus::LEVEL_OK) {
        status.level = robot_arm_msgs::msg::SafetyStatus::LEVEL_WARN;
        status.message = "approaching a joint limit";
      }
    }

    if (e_stop_active_) {
      status.level = robot_arm_msgs::msg::SafetyStatus::LEVEL_ESTOP;
      if (status.message.empty()) {
        status.message = "emergency stop engaged";
      }
    }

    latest_status_ = status;
    safety_publisher_->publish(status);
  }

  // -- actions ------------------------------------------------------------

  void engage_e_stop(const std::string & reason)
  {
    e_stop_active_ = true;
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP engaged (%s)", reason.c_str());

    std_msgs::msg::Bool message;
    message.data = true;
    e_stop_publisher_->publish(message);

    if (hardware_e_stop_client_->service_is_ready()) {
      auto request = std::make_shared<robot_arm_interfaces::srv::SetEStop::Request>();
      request->engage = true;
      request->reason = reason;
      hardware_e_stop_client_->async_send_request(request);
    }

    if (stop_controller_on_estop_) {
      // Deactivating the trajectory controller aborts the running trajectory,
      // so it cannot resume when the stop is released.
      switch_controllers({}, {arm_controller_});
    }
  }

  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate)
  {
    if (!switch_client_->wait_for_service(1s)) {
      RCLCPP_WARN(
        get_logger(), "%s/switch_controller is not available",
        controller_manager_.c_str());
      return false;
    }

    auto request =
      std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness =
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

    auto future = switch_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "switch_controller did not answer in time");
      return false;
    }
    return future.get()->ok;
  }

  // -- services -----------------------------------------------------------

  void handle_set_e_stop(
    const std::shared_ptr<robot_arm_interfaces::srv::SetEStop::Request> request,
    std::shared_ptr<robot_arm_interfaces::srv::SetEStop::Response> response)
  {
    if (request->engage) {
      engage_e_stop(request->reason.empty() ? "service call" : request->reason);
      response->success = true;
      response->message = "emergency stop engaged, trajectory execution aborted";
    } else {
      e_stop_active_ = false;
      consecutive_violations_ = 0;

      std_msgs::msg::Bool message;
      message.data = false;
      e_stop_publisher_->publish(message);

      if (hardware_e_stop_client_->service_is_ready()) {
        auto forwarded = std::make_shared<robot_arm_interfaces::srv::SetEStop::Request>();
        forwarded->engage = false;
        hardware_e_stop_client_->async_send_request(forwarded);
      }
      RCLCPP_WARN(
        get_logger(), "Emergency stop released; enable the arm again to resume motion");
      response->success = true;
      response->message = "emergency stop released; call set_motor_enable to resume";
    }
    response->e_stop_active = e_stop_active_;
  }

  void handle_set_motor_enable(
    const std::shared_ptr<robot_arm_interfaces::srv::SetMotorEnable::Request> request,
    std::shared_ptr<robot_arm_interfaces::srv::SetMotorEnable::Response> response)
  {
    if (request->enable && e_stop_active_) {
      response->success = false;
      response->enabled = false;
      response->message = "refusing to enable while the emergency stop is engaged";
      return;
    }

    // On the real robot this means power; in simulation the closest
    // equivalent is whether the trajectory controller is active.  Both are
    // reached through the same service name, which is what keeps the user
    // API identical in the two modes.
    if (hardware_enable_client_->service_is_ready()) {
      auto forwarded = std::make_shared<robot_arm_interfaces::srv::SetMotorEnable::Request>();
      forwarded->enable = request->enable;
      auto future = hardware_enable_client_->async_send_request(forwarded);
      if (future.wait_for(3s) == std::future_status::ready) {
        const auto result = future.get();
        response->success = result->success;
        response->enabled = result->enabled;
        response->message = "hardware: " + result->message;
        if (result->success && request->enable) {
          switch_controllers({arm_controller_}, {});
        }
        return;
      }
      response->success = false;
      response->message = "the hardware driver did not answer";
      return;
    }

    const bool ok = request->enable ?
      switch_controllers({arm_controller_}, {}) :
      switch_controllers({}, {arm_controller_});
    response->success = ok;
    response->enabled = ok ? request->enable : !request->enable;
    response->message = ok ?
      (std::string("simulation: ") + arm_controller_ +
      (request->enable ? " activated" : " deactivated")) :
      "could not switch the controller";
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Deactivating aborts the running trajectory; re-activating leaves the
    // controller holding the pose the arm is in right now, with no jump.
    const bool stopped = switch_controllers({}, {arm_controller_});
    const bool restarted = stopped && switch_controllers({arm_controller_}, {});

    response->success = stopped && restarted;
    response->message = response->success ?
      "trajectory aborted, holding the current pose" :
      "could not restart the controller after stopping";
    RCLCPP_WARN(get_logger(), "Stop requested: %s", response->message.c_str());
  }

  // -- diagnostics --------------------------------------------------------

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus entry;
    entry.name = "robot_arm: safety monitor";
    entry.hardware_id = "robot_arm";

    switch (latest_status_.level) {
      case robot_arm_msgs::msg::SafetyStatus::LEVEL_OK:
        entry.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        entry.message = "within limits";
        break;
      case robot_arm_msgs::msg::SafetyStatus::LEVEL_WARN:
        entry.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        entry.message = latest_status_.message;
        break;
      default:
        entry.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        entry.message = latest_status_.message;
        break;
    }

    auto add = [&entry](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = value;
        entry.values.push_back(pair);
      };
    add("e_stop", latest_status_.e_stop_active ? "engaged" : "released");
    add("position_limit_violation", latest_status_.position_limit_violation ? "true" : "false");
    add("velocity_limit_violation", latest_status_.velocity_limit_violation ? "true" : "false");
    add("effort_limit_violation", latest_status_.effort_limit_violation ? "true" : "false");
    add("joint_states_age_s", last_joint_state_time_.nanoseconds() == 0 ? "never" :
      std::to_string((now() - last_joint_state_time_).seconds()));
    add("supervised_joints", std::to_string(limits_.size()));

    std::string violating;
    for (const auto & joint : latest_status_.violating_joints) {
      violating += (violating.empty() ? "" : ", ") + joint;
    }
    add("violating_joints", violating);

    array.status.push_back(entry);
    diagnostics_publisher_->publish(array);
  }

  // -- members ------------------------------------------------------------

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::vector<std::string> joint_names_;
  std::map<std::string, JointLimits> limits_;
  std::string controller_manager_;
  std::string arm_controller_;
  std::string hardware_node_;
  double check_rate_{20.0};
  double joint_state_timeout_{0.5};
  double diagnostics_period_{1.0};
  double velocity_scale_{1.0};
  double warn_margin_{0.05};
  bool stop_controller_on_estop_{true};
  bool estop_on_violation_{true};
  int violation_cycles_{3};

  sensor_msgs::msg::JointState last_joint_state_;
  rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
  robot_arm_msgs::msg::SafetyStatus latest_status_;
  bool e_stop_active_{false};
  int consecutive_violations_{0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_publisher_;
  rclcpp::Publisher<robot_arm_msgs::msg::SafetyStatus>::SharedPtr safety_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<robot_arm_interfaces::srv::SetMotorEnable>::SharedPtr hardware_enable_client_;
  rclcpp::Client<robot_arm_interfaces::srv::SetEStop>::SharedPtr hardware_e_stop_client_;
  rclcpp::Service<robot_arm_interfaces::srv::SetEStop>::SharedPtr e_stop_service_;
  rclcpp::Service<robot_arm_interfaces::srv::SetMotorEnable>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace robot_arm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Services call other services from inside their own callbacks, so the
  // executor must be able to run more than one callback at a time.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<robot_arm_control::SafetyMonitor>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
