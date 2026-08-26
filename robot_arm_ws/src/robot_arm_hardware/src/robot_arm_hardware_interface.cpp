// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/robot_arm_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "robot_arm_hardware/parameter_utils.hpp"

namespace robot_arm_hardware
{
namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kInitialFeedbackAttempts = 20;
}  // namespace

double RobotArmSystemHardware::now_seconds()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ===========================================================================
//  Configuration
// ===========================================================================

bool RobotArmSystemHardware::parse_hardware_parameters(std::string & error)
{
  const auto & params = info_.hardware_parameters;
  try {
    transport_config_.type = get_string(params, "transport_type", "loopback");
    transport_config_.serial_port = get_string(params, "serial_port", "/dev/ttyUSB0");
    transport_config_.baudrate = get_int(params, "serial_baudrate", 921600);
    transport_config_.parity = get_string(params, "serial_parity", "none");
    transport_config_.data_bits = get_int(params, "serial_data_bits", 8);
    transport_config_.stop_bits = get_int(params, "serial_stop_bits", 1);
    transport_config_.rs485_rts_toggle = get_bool(params, "rs485_rts_toggle", false);
    transport_config_.can_interface = get_string(params, "can_interface", "can0");
    transport_config_.can_base_id = get_int(params, "can_base_id", 0x100);
    transport_config_.tcp_host = get_string(params, "tcp_host", "127.0.0.1");
    transport_config_.tcp_port = get_int(params, "tcp_port", 5000);
    transport_config_.read_timeout_ms = get_int(params, "read_timeout_ms", 8);
    transport_config_.write_timeout_ms = get_int(params, "write_timeout_ms", 8);

    protocol_config_.type = get_string(params, "protocol_type", "loopback");
    protocol_config_.checksum = get_bool(params, "protocol_checksum", true);
    protocol_config_.can_base_id = transport_config_.can_base_id;

    safety_limits_.command_timeout = get_double(params, "command_timeout", 0.25);
    safety_limits_.comm_timeout = get_double(params, "comm_timeout", 0.20);
    safety_limits_.velocity_scale = get_double(params, "velocity_scale", 1.0);
    safety_limits_.position_margin = get_double(params, "position_margin", 0.0);
    safety_limits_.clamp_commands = get_bool(params, "clamp_commands", true);

    max_consecutive_errors_ = get_int(params, "max_consecutive_errors", 5);
    safety_limits_.max_consecutive_errors = max_consecutive_errors_;

    enable_on_activate_ = get_bool(params, "enable_on_activate", true);
    diagnostics_period_ = get_double(params, "diagnostics_period", 1.0);
    node_namespace_ = get_string(params, "node_namespace", "");
    calibration_file_ = get_string(params, "calibration_file", "");
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }

  if (safety_limits_.command_timeout <= 0.0 || safety_limits_.comm_timeout <= 0.0) {
    error = "command_timeout and comm_timeout must be > 0";
    return false;
  }
  if (diagnostics_period_ <= 0.0) {
    error = "diagnostics_period must be > 0";
    return false;
  }
  return true;
}

bool RobotArmSystemHardware::parse_joint_parameters(std::string & error)
{
  const bool apply_gear_ratio =
    get_bool(info_.hardware_parameters, "apply_gear_ratio_in_driver", true);

  joint_configs_.clear();
  joint_configs_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    // The controllers rely on these interfaces being present; failing early
    // with a clear message beats a mysterious mismatch at activation time.
    const bool has_position_command = std::any_of(
      joint.command_interfaces.begin(), joint.command_interfaces.end(),
      [](const auto & interface) {return interface.name == hardware_interface::HW_IF_POSITION;});
    if (!has_position_command) {
      error = "joint '" + joint.name + "' has no position command interface";
      return false;
    }

    for (const auto & required :
      {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY})
    {
      const bool present = std::any_of(
        joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [&required](const auto & interface) {return interface.name == required;});
      if (!present) {
        error = "joint '" + joint.name + "' has no '" + required + "' state interface";
        return false;
      }
    }

    try {
      joint_configs_.push_back(
        joint_config_from_parameters(joint.name, joint.parameters, apply_gear_ratio));
    } catch (const std::exception & exception) {
      error = std::string("invalid configuration for joint '") + joint.name + "': " +
        exception.what();
      return false;
    }
  }

  if (joint_configs_.empty()) {
    error = "no joints configured";
    return false;
  }
  return true;
}

// ===========================================================================
//  Lifecycle
// ===========================================================================

hardware_interface::CallbackReturn RobotArmSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string error;
  if (!parse_hardware_parameters(error)) {
    RCLCPP_ERROR(logger_, "Invalid hardware configuration: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!parse_joint_parameters(error)) {
    RCLCPP_ERROR(logger_, "Invalid joint configuration: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t count = joint_configs_.size();
  hw_positions_.assign(count, kNaN);
  hw_velocities_.assign(count, kNaN);
  hw_efforts_.assign(count, kNaN);
  hw_position_commands_.assign(count, kNaN);
  hw_velocity_commands_.assign(count, 0.0);

  last_counts_.assign(count, 0);
  last_current_.assign(count, kNaN);
  last_temperature_.assign(count, kNaN);
  last_fault_.assign(count, 0);
  last_valid_.assign(count, false);
  control_modes_.assign(count, ControlMode::kPosition);

  safety_.configure(joint_configs_, safety_limits_);

  transport_ = create_transport(transport_config_, error);
  if (!transport_) {
    RCLCPP_ERROR(logger_, "%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  protocol_ = create_protocol(protocol_config_, error);
  if (!protocol_) {
    RCLCPP_ERROR(logger_, "%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_, "Configured %zu joints, transport '%s', protocol '%s'",
    count, transport_->name().c_str(), protocol_->name().c_str());
  for (const auto & joint : joint_configs_) {
    RCLCPP_INFO(
      logger_,
      "  %s: motor id %d, %ld counts/rev, gear %.2f, limits [%.3f, %.3f] rad",
      joint.name.c_str(), joint.motor_id, static_cast<long>(joint.encoder_resolution),
      joint.gear_ratio, joint.min_position, joint.max_position);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;
  if (!transport_->open(error)) {
    RCLCPP_ERROR(logger_, "Cannot open %s: %s", transport_->name().c_str(), error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!protocol_->initialize(transport_.get(), joint_configs_, protocol_config_, error)) {
    RCLCPP_ERROR(logger_, "Cannot initialise protocol: %s", error.c_str());
    transport_->close();
    return hardware_interface::CallbackReturn::ERROR;
  }
  connected_ = true;
  RCLCPP_INFO(logger_, "Robot connected via %s", transport_->name().c_str());

  // Seed the state from the encoders, so the first command the controller
  // sends is relative to where the arm actually is - never to a default pose.
  std::vector<MotorFeedback> feedback;
  bool seeded = false;
  for (int attempt = 0; attempt < kInitialFeedbackAttempts && !seeded; ++attempt) {
    if (protocol_->read_feedback(feedback, error) && feedback.size() == joint_configs_.size()) {
      seeded = std::all_of(
        feedback.begin(), feedback.end(), [](const MotorFeedback & item) {return item.valid;});
    }
    if (!seeded) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  if (!seeded) {
    RCLCPP_ERROR(logger_, "No valid encoder feedback after configure: %s", error.c_str());
    transport_->close();
    connected_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
      hw_positions_[i] = joint_configs_[i].counts_to_position(feedback[i].counts);
      hw_velocities_[i] = 0.0;
      hw_efforts_[i] = 0.0;
      hw_position_commands_[i] = hw_positions_[i];
      hw_velocity_commands_[i] = 0.0;
      last_counts_[i] = feedback[i].counts;
      last_valid_[i] = true;
    }
  }
  RCLCPP_INFO(logger_, "Encoder feedback received, joint states initialised");

  start_node();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const double now = now_seconds();
  safety_.notify_command(now);
  safety_.notify_feedback(now);
  safety_.notify_success();
  last_read_time_ = now;
  last_write_time_ = now;
  last_valid_feedback_time_ = now;
  read_errors_ = 0;
  write_errors_ = 0;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
      hw_position_commands_[i] = hw_positions_[i];
      hw_velocity_commands_[i] = 0.0;
    }
  }

  if (safety_.e_stop_active()) {
    RCLCPP_WARN(logger_, "Activating with the emergency stop engaged: motion stays blocked");
  } else if (enable_on_activate_) {
    std::string error;
    if (!set_motors_enabled(true, error)) {
      RCLCPP_ERROR(logger_, "Cannot enable the drives: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  active_ = true;
  RCLCPP_INFO(logger_, "Joint controllers active, hardware ready");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  std::string error;
  if (protocol_ && !protocol_->stop(error)) {
    RCLCPP_WARN(logger_, "Stop command failed on deactivate: %s", error.c_str());
  }
  if (!set_motors_enabled(false, error)) {
    RCLCPP_WARN(logger_, "Cannot disable the drives: %s", error.c_str());
  }
  RCLCPP_INFO(logger_, "Hardware deactivated, drives disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_node();
  if (transport_) {
    transport_->close();
  }
  connected_ = false;
  RCLCPP_INFO(logger_, "Disconnected from the motor controller");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::CallbackReturn RobotArmSystemHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reaching the error state means the arm must not move any more.
  active_ = false;
  std::string error;
  if (protocol_) {
    protocol_->stop(error);
    protocol_->enable(false, error);
  }
  motors_enabled_ = false;
  RCLCPP_ERROR(logger_, "Hardware entered the error state, drives disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===========================================================================
//  Interfaces
// ===========================================================================

std::vector<hardware_interface::StateInterface>
RobotArmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    interfaces.emplace_back(
      joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(
      joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    interfaces.emplace_back(
      joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotArmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    interfaces.emplace_back(
      joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
    interfaces.emplace_back(
      joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
  }
  return interfaces;
}

hardware_interface::return_type RobotArmSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  // A joint may be driven in position OR velocity, never in both at once.
  for (const auto & joint : joint_configs_) {
    const bool position = std::find(
      start_interfaces.begin(), start_interfaces.end(),
      joint.name + "/" + hardware_interface::HW_IF_POSITION) != start_interfaces.end();
    const bool velocity = std::find(
      start_interfaces.begin(), start_interfaces.end(),
      joint.name + "/" + hardware_interface::HW_IF_VELOCITY) != start_interfaces.end();
    if (position && velocity) {
      RCLCPP_ERROR(
        logger_, "Joint %s cannot be claimed for position and velocity at the same time",
        joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    const std::string position_key =
      joint_configs_[i].name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string velocity_key =
      joint_configs_[i].name + "/" + hardware_interface::HW_IF_VELOCITY;

    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), velocity_key) !=
      stop_interfaces.end())
    {
      control_modes_[i] = ControlMode::kPosition;
      hw_velocity_commands_[i] = 0.0;
    }
    if (std::find(start_interfaces.begin(), start_interfaces.end(), velocity_key) !=
      start_interfaces.end())
    {
      control_modes_[i] = ControlMode::kVelocity;
    }
    if (std::find(start_interfaces.begin(), start_interfaces.end(), position_key) !=
      start_interfaces.end())
    {
      control_modes_[i] = ControlMode::kPosition;
      hw_position_commands_[i] = hw_positions_[i];
    }
  }
  return hardware_interface::return_type::OK;
}

// ===========================================================================
//  Control loop
// ===========================================================================

hardware_interface::return_type RobotArmSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const double now = now_seconds();
  last_read_time_ = now;

  std::vector<MotorFeedback> feedback;
  std::string error;
  if (!protocol_->read_feedback(feedback, error) || feedback.size() != joint_configs_.size()) {
    ++read_errors_;
    safety_.notify_error();
    if (safety_.consecutive_errors() >= max_consecutive_errors_) {
      RCLCPP_ERROR(
        logger_, "Motor controller communication lost (%d consecutive read errors): %s",
        safety_.consecutive_errors(), error.c_str());
      std::string ignored;
      protocol_->stop(ignored);
      protocol_->enable(false, ignored);
      motors_enabled_ = false;
      connected_ = false;
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 1000, "Encoder read failed, retrying: %s", error.c_str());
    if (transport_) {
      transport_->flush();
    }
    return hardware_interface::return_type::OK;
  }

  std::vector<double> positions(joint_configs_.size(), kNaN);
  std::vector<double> velocities(joint_configs_.size(), kNaN);
  std::vector<double> efforts(joint_configs_.size(), kNaN);
  bool all_valid = true;

  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto & joint = joint_configs_[i];
    const auto & item = feedback[i];

    if (!item.valid || !joint.is_plausible_counts(item.counts)) {
      all_valid = false;
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 1000,
        "Invalid encoder value on %s (raw %ld)", joint.name.c_str(),
        static_cast<long>(item.counts));
      continue;
    }
    positions[i] = joint.counts_to_position(item.counts);
    velocities[i] = joint.counts_to_velocity(item.counts_per_s);
    efforts[i] = std::isfinite(item.current) ? joint.current_to_effort(item.current) : kNaN;
  }

  SafetyReport report;
  if (!all_valid || !safety_.validate_feedback(positions, velocities, report)) {
    ++read_errors_;
    safety_.notify_error();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_report_ = report;
    }
    if (safety_.consecutive_errors() >= max_consecutive_errors_) {
      RCLCPP_ERROR(logger_, "Too many invalid encoder readings, stopping the arm");
      std::string ignored;
      protocol_->stop(ignored);
      protocol_->enable(false, ignored);
      motors_enabled_ = false;
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
      hw_positions_[i] = positions[i];
      hw_velocities_[i] = velocities[i];
      hw_efforts_[i] = efforts[i];
      last_counts_[i] = feedback[i].counts;
      last_current_[i] = feedback[i].current;
      last_temperature_[i] = feedback[i].temperature;
      last_fault_[i] = feedback[i].fault_code;
      last_valid_[i] = true;
    }
  }

  safety_.notify_feedback(now);
  safety_.notify_success();
  last_valid_feedback_time_ = now;
  connected_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double now = now_seconds();
  last_write_time_ = now;
  safety_.notify_command(now);

  std::vector<double> position_commands;
  std::vector<double> velocity_commands;
  std::vector<double> measured;
  std::vector<ControlMode> modes;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (watchdog_tripped_.exchange(false)) {
      // The control loop was stalled: throw away whatever setpoint was left
      // over and restart from the measured pose.
      for (std::size_t i = 0; i < hw_position_commands_.size(); ++i) {
        hw_position_commands_[i] = hw_positions_[i];
        hw_velocity_commands_[i] = 0.0;
      }
      RCLCPP_WARN(logger_, "Control loop resumed; commands re-seeded from the encoders");
    }
    position_commands = hw_position_commands_;
    velocity_commands = hw_velocity_commands_;
    measured = hw_positions_;
    modes = control_modes_;
  }

  SafetyReport report = safety_.check_commands(
    position_commands, velocity_commands, measured, period.seconds(), now);
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_report_ = report;
  }

  if (report.level >= SafetyReport::Level::kViolation && !report.violating_joints.empty()) {
    std::string joints;
    for (const auto & name : report.violating_joints) {
      joints += (joints.empty() ? "" : ", ") + name;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 1000, "Safety layer acted on [%s]: %s",
      joints.c_str(), report.message.c_str());
  }

  std::string error;
  if (!report.motion_allowed || !motors_enabled_.load()) {
    // Never keep driving with the last setpoint: command a controlled stop
    // once, then stay quiet until motion is allowed again.
    if (!motion_blocked_.exchange(true)) {
      if (protocol_ && !protocol_->stop(error)) {
        RCLCPP_WARN(logger_, "Stop command failed: %s", error.c_str());
      }
      RCLCPP_WARN(logger_, "Motion blocked: %s", report.message.c_str());
    }
    return hardware_interface::return_type::OK;
  }
  motion_blocked_ = false;

  std::vector<MotorCommand> commands;
  commands.reserve(joint_configs_.size());
  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto & joint = joint_configs_[i];
    MotorCommand command;
    command.motor_id = joint.motor_id;
    command.velocity_mode = modes[i] == ControlMode::kVelocity;
    command.target_counts = static_cast<double>(joint.position_to_counts(position_commands[i]));
    command.target_counts_per_s = joint.velocity_to_counts(velocity_commands[i]);
    commands.push_back(command);
  }

  if (!protocol_->write_commands(commands, error)) {
    ++write_errors_;
    safety_.notify_error();
    if (safety_.consecutive_errors() >= max_consecutive_errors_) {
      RCLCPP_ERROR(logger_, "Motor controller communication lost on write: %s", error.c_str());
      std::string ignored;
      protocol_->stop(ignored);
      protocol_->enable(false, ignored);
      motors_enabled_ = false;
      connected_ = false;
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 1000, "Command write failed, retrying: %s", error.c_str());
    return hardware_interface::return_type::OK;
  }

  safety_.notify_success();
  return hardware_interface::return_type::OK;
}

// ===========================================================================
//  Helpers
// ===========================================================================

bool RobotArmSystemHardware::set_motors_enabled(bool enabled, std::string & error)
{
  if (!protocol_) {
    error = "protocol is not initialised";
    return false;
  }
  if (enabled && safety_.e_stop_active()) {
    error = "refusing to enable the drives while the emergency stop is engaged";
    return false;
  }
  if (!protocol_->enable(enabled, error)) {
    return false;
  }
  motors_enabled_ = enabled;
  RCLCPP_INFO(logger_, "Drives %s", enabled ? "enabled" : "disabled");
  return true;
}

bool RobotArmSystemHardware::engage_e_stop(const std::string & reason)
{
  safety_.set_e_stop(true, reason);
  std::string error;
  bool ok = true;
  if (protocol_) {
    if (!protocol_->stop(error)) {
      RCLCPP_ERROR(logger_, "E-STOP: stop command failed: %s", error.c_str());
      ok = false;
    }
    if (!protocol_->enable(false, error)) {
      RCLCPP_ERROR(logger_, "E-STOP: disabling the drives failed: %s", error.c_str());
      ok = false;
    }
  }
  motors_enabled_ = false;
  motion_blocked_ = true;
  RCLCPP_ERROR(logger_, "EMERGENCY STOP engaged (%s)", reason.c_str());
  return ok;
}

robot_arm_msgs::msg::ArmStatus RobotArmSystemHardware::build_status_message() const
{
  robot_arm_msgs::msg::ArmStatus status;
  status.connected = connected_.load();
  status.enabled = motors_enabled_.load();
  status.e_stop_active = safety_.e_stop_active();
  status.transport = transport_ ? transport_->name() : "";
  status.protocol = protocol_ ? protocol_->name() : "";
  status.read_errors = read_errors_.load();
  status.write_errors = write_errors_.load();

  const double last_feedback = last_valid_feedback_time_.load();
  status.last_comm_age = last_feedback >= 0.0 ? now_seconds() - last_feedback :
    std::numeric_limits<double>::infinity();
  status.communication_ok = status.last_comm_age <= safety_limits_.comm_timeout;

  std::lock_guard<std::mutex> lock(data_mutex_);
  bool any_velocity = false;
  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    robot_arm_msgs::msg::JointDiagnostic diagnostic;
    diagnostic.name = joint_configs_[i].name;
    diagnostic.position = hw_positions_[i];
    diagnostic.velocity = hw_velocities_[i];
    diagnostic.effort = hw_efforts_[i];
    diagnostic.temperature = last_temperature_[i];
    diagnostic.current = last_current_[i];
    diagnostic.raw_encoder = last_counts_[i];
    diagnostic.enabled = motors_enabled_.load();
    diagnostic.fault_code = last_fault_[i];
    diagnostic.fault_message = last_fault_[i] == 0 ? "" : "drive reported a fault";
    status.joints.push_back(diagnostic);
    any_velocity = any_velocity || control_modes_[i] == ControlMode::kVelocity;
  }
  status.control_mode = !active_.load() ? "idle" : (any_velocity ? "velocity" : "position");
  return status;
}

bool RobotArmSystemHardware::write_calibration_file(
  const std::string & path, std::string & error) const
{
  std::ofstream file(path);
  if (!file) {
    error = "cannot open '" + path + "' for writing";
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  file << std::fixed << std::setprecision(6);
  file << "# robot_arm - joint calibration\n";
  file << "# Written by robot_arm_hardware; edit by hand at your own risk.\n";
  file << "calibration:\n";
  for (const auto & joint : joint_configs_) {
    file << "  " << joint.name << ":\n";
    file << "    zero_offset: " << joint.zero_offset << "\n";
    file << "    direction: " << joint.direction << "\n";
    file << "    min_position: " << joint.min_position << "\n";
    file << "    max_position: " << joint.max_position << "\n";
    file << "    home_position: " << joint.home_position << "\n";
  }
  if (!file) {
    error = "write to '" + path + "' failed";
    return false;
  }
  return true;
}

// ===========================================================================
//  ROS interface of the driver
//
//  The hardware plugin is loaded by the controller_manager and therefore has
//  no node of its own.  It spins a small one here so that the e-stop, the
//  service interface and the diagnostics are available in every launch mode -
//  without ever blocking the real-time read()/write() path.
// ===========================================================================

void RobotArmSystemHardware::start_node()
{
  if (node_) {
    return;
  }
  if (!rclcpp::ok()) {
    RCLCPP_WARN(
      logger_, "No ROS context: running without the e-stop, service and diagnostics interface");
    return;
  }

  node_ = std::make_shared<rclcpp::Node>("robot_arm_hardware", node_namespace_);

  // Deliberately default (volatile) QoS so that a plain
  //   ros2 topic pub /e_stop std_msgs/msg/Bool "data: true"
  // reaches the driver.  The service below is the acknowledged path.
  e_stop_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/e_stop", rclcpp::QoS(10),
    std::bind(&RobotArmSystemHardware::on_e_stop_message, this, std::placeholders::_1));

  status_publisher_ =
    node_->create_publisher<robot_arm_msgs::msg::ArmStatus>("~/status", rclcpp::QoS(10));
  safety_publisher_ =
    node_->create_publisher<robot_arm_msgs::msg::SafetyStatus>("~/safety_status", rclcpp::QoS(10));
  diagnostics_publisher_ =
    node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));

  e_stop_service_ = node_->create_service<robot_arm_interfaces::srv::SetEStop>(
    "~/set_e_stop",
    std::bind(
      &RobotArmSystemHardware::handle_set_e_stop, this,
      std::placeholders::_1, std::placeholders::_2));
  enable_service_ = node_->create_service<robot_arm_interfaces::srv::SetMotorEnable>(
    "~/set_motor_enable",
    std::bind(
      &RobotArmSystemHardware::handle_set_motor_enable, this,
      std::placeholders::_1, std::placeholders::_2));
  calibrate_service_ = node_->create_service<robot_arm_interfaces::srv::CalibrateJoint>(
    "~/calibrate_joint",
    std::bind(
      &RobotArmSystemHardware::handle_calibrate_joint, this,
      std::placeholders::_1, std::placeholders::_2));
  save_service_ = node_->create_service<robot_arm_interfaces::srv::SaveCalibration>(
    "~/save_calibration",
    std::bind(
      &RobotArmSystemHardware::handle_save_calibration, this,
      std::placeholders::_1, std::placeholders::_2));
  get_calibration_service_ = node_->create_service<robot_arm_interfaces::srv::GetCalibration>(
    "~/get_calibration",
    std::bind(
      &RobotArmSystemHardware::handle_get_calibration, this,
      std::placeholders::_1, std::placeholders::_2));

  diagnostics_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(diagnostics_period_),
    std::bind(&RobotArmSystemHardware::publish_diagnostics, this));

  // The watchdog must sample several times per timeout to be useful.
  const double watchdog_period = std::max(0.02, safety_limits_.command_timeout / 4.0);
  watchdog_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(watchdog_period),
    std::bind(&RobotArmSystemHardware::watchdog, this));

  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  node_running_ = true;
  node_thread_ = std::thread(
    [this]() {
      while (node_running_ && rclcpp::ok()) {
        executor_->spin_some(std::chrono::milliseconds(20));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    });

  RCLCPP_INFO(logger_, "Driver node started (namespace '%s')", node_->get_namespace());
}

void RobotArmSystemHardware::stop_node()
{
  node_running_ = false;
  if (node_thread_.joinable()) {
    node_thread_.join();
  }
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
  diagnostics_timer_.reset();
  watchdog_timer_.reset();
  e_stop_subscription_.reset();
  status_publisher_.reset();
  safety_publisher_.reset();
  diagnostics_publisher_.reset();
  e_stop_service_.reset();
  enable_service_.reset();
  calibrate_service_.reset();
  save_service_.reset();
  get_calibration_service_.reset();
  executor_.reset();
  node_.reset();
}

void RobotArmSystemHardware::watchdog()
{
  if (!active_.load()) {
    return;
  }
  const double now = now_seconds();

  const double write_time = last_write_time_.load();
  if (write_time >= 0.0 && (now - write_time) > safety_limits_.command_timeout &&
    !watchdog_tripped_.load())
  {
    watchdog_tripped_ = true;
    motion_blocked_ = true;
    RCLCPP_ERROR(
      logger_, "Controller timeout: no write() for %.3f s, stopping the arm",
      now - write_time);
    std::string error;
    if (protocol_ && !protocol_->stop(error)) {
      RCLCPP_ERROR(logger_, "Stop command failed during watchdog: %s", error.c_str());
    }
    // Power is deliberately kept so the arm holds its pose instead of
    // collapsing under gravity; motion stays blocked until write() resumes.
  }

  const double feedback_time = last_valid_feedback_time_.load();
  if (feedback_time >= 0.0 && (now - feedback_time) > safety_limits_.comm_timeout) {
    RCLCPP_ERROR_THROTTLE(
      logger_, steady_clock_, 1000,
      "Motor controller communication lost: no valid feedback for %.3f s",
      now - feedback_time);
  }
}

void RobotArmSystemHardware::publish_diagnostics()
{
  if (!node_) {
    return;
  }

  const auto status = build_status_message();
  auto stamped = status;
  stamped.header.stamp = node_->now();
  stamped.header.frame_id = "base_link";
  status_publisher_->publish(stamped);

  SafetyReport report;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    report = last_report_;
  }

  robot_arm_msgs::msg::SafetyStatus safety;
  safety.header = stamped.header;
  safety.level = static_cast<uint8_t>(report.level);
  safety.e_stop_active = report.e_stop_active || safety_.e_stop_active();
  safety.position_limit_violation = report.position_limit_violation;
  safety.velocity_limit_violation = report.velocity_limit_violation;
  safety.effort_limit_violation = report.effort_limit_violation;
  safety.command_timeout = report.command_timeout || watchdog_tripped_.load();
  safety.communication_timeout = report.communication_timeout || !stamped.communication_ok;
  safety.invalid_feedback = report.invalid_feedback;
  safety.violating_joints = report.violating_joints;
  safety.message = report.message;
  safety_publisher_->publish(safety);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header = stamped.header;

  auto add_value = [](diagnostic_msgs::msg::DiagnosticStatus & entry,
      const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      entry.values.push_back(pair);
    };
  auto number = [](double value) {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(3) << value;
      return stream.str();
    };

  // --- communication -------------------------------------------------------
  diagnostic_msgs::msg::DiagnosticStatus communication;
  communication.name = "robot_arm: communication";
  communication.hardware_id = stamped.transport;
  if (!stamped.connected) {
    communication.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    communication.message = "not connected";
  } else if (!stamped.communication_ok) {
    communication.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    communication.message = "motor controller communication lost";
  } else {
    communication.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    communication.message = "connected";
  }
  add_value(communication, "transport", stamped.transport);
  add_value(communication, "protocol", stamped.protocol);
  add_value(communication, "last_reply_age_s", number(stamped.last_comm_age));
  add_value(communication, "read_errors", std::to_string(stamped.read_errors));
  add_value(communication, "write_errors", std::to_string(stamped.write_errors));
  array.status.push_back(communication);

  // --- controller / safety -------------------------------------------------
  diagnostic_msgs::msg::DiagnosticStatus controller;
  controller.name = "robot_arm: controller";
  controller.hardware_id = "robot_arm";
  if (safety.e_stop_active) {
    controller.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    controller.message = "emergency stop engaged";
  } else if (report.level == SafetyReport::Level::kViolation) {
    controller.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    controller.message = report.message.empty() ? "safety limit reached" : report.message;
  } else {
    controller.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    controller.message = active_.load() ? "active" : "idle";
  }
  add_value(controller, "state", stamped.control_mode);
  add_value(controller, "drives_enabled", stamped.enabled ? "true" : "false");
  add_value(controller, "e_stop", safety.e_stop_active ? "engaged" : "released");
  add_value(controller, "safety_level", to_string(report.level));
  array.status.push_back(controller);

  // --- one entry per joint -------------------------------------------------
  for (std::size_t i = 0; i < stamped.joints.size(); ++i) {
    const auto & joint = stamped.joints[i];
    const auto & config = joint_configs_[i];

    diagnostic_msgs::msg::DiagnosticStatus entry;
    entry.name = "robot_arm: " + joint.name;
    entry.hardware_id = "motor_" + std::to_string(config.motor_id);
    entry.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    entry.message = "nominal";

    if (joint.fault_code != 0) {
      entry.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      entry.message = "drive fault " + std::to_string(joint.fault_code);
    } else if (std::isfinite(joint.temperature) && joint.temperature > config.max_temperature) {
      entry.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      entry.message = "temperature high";
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 5000, "%s temperature high: %.1f degC (limit %.1f)",
        joint.name.c_str(), joint.temperature, config.max_temperature);
    } else if (std::isfinite(joint.current) && config.max_current > 0.0 &&
      std::abs(joint.current) > config.max_current)
    {
      entry.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      entry.message = "current high";
    }

    add_value(entry, "position_rad", number(joint.position));
    add_value(entry, "velocity_rad_s", number(joint.velocity));
    add_value(entry, "effort_Nm", number(joint.effort));
    add_value(entry, "current_A", number(joint.current));
    add_value(entry, "temperature_C", number(joint.temperature));
    add_value(entry, "raw_encoder", std::to_string(joint.raw_encoder));
    add_value(entry, "limit_min_rad", number(config.min_position));
    add_value(entry, "limit_max_rad", number(config.max_position));
    array.status.push_back(entry);
  }

  diagnostics_publisher_->publish(array);
}

// ---------------------------------------------------------------------------
//  Callbacks
// ---------------------------------------------------------------------------

void RobotArmSystemHardware::on_e_stop_message(const std_msgs::msg::Bool::SharedPtr message)
{
  if (message->data) {
    if (!safety_.e_stop_active()) {
      engage_e_stop("/e_stop topic");
    }
  } else if (safety_.e_stop_active()) {
    safety_.set_e_stop(false, "");
    RCLCPP_WARN(
      logger_,
      "Emergency stop released; the drives stay disabled until they are enabled explicitly");
  }
}

void RobotArmSystemHardware::handle_set_e_stop(
  const std::shared_ptr<robot_arm_interfaces::srv::SetEStop::Request> request,
  std::shared_ptr<robot_arm_interfaces::srv::SetEStop::Response> response)
{
  if (request->engage) {
    const bool ok = engage_e_stop(request->reason.empty() ? "service call" : request->reason);
    response->success = ok;
    response->message = ok ? "emergency stop engaged" :
      "emergency stop engaged, but the drives did not acknowledge";
  } else {
    safety_.set_e_stop(false, "");
    response->success = true;
    response->message = "emergency stop released; enable the drives to resume motion";
    RCLCPP_WARN(logger_, "Emergency stop released via service");
  }
  response->e_stop_active = safety_.e_stop_active();
}

void RobotArmSystemHardware::handle_set_motor_enable(
  const std::shared_ptr<robot_arm_interfaces::srv::SetMotorEnable::Request> request,
  std::shared_ptr<robot_arm_interfaces::srv::SetMotorEnable::Response> response)
{
  std::string error;
  response->success = set_motors_enabled(request->enable, error);
  response->enabled = motors_enabled_.load();
  response->message = response->success ?
    (request->enable ? "drives enabled" : "drives disabled") : error;
}

void RobotArmSystemHardware::handle_calibrate_joint(
  const std::shared_ptr<robot_arm_interfaces::srv::CalibrateJoint::Request> request,
  std::shared_ptr<robot_arm_interfaces::srv::CalibrateJoint::Response> response)
{
  if (motors_enabled_.load()) {
    response->success = false;
    response->message = "disable the drives before calibrating";
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  for (std::size_t i = 0; i < joint_configs_.size(); ++i) {
    auto & joint = joint_configs_[i];
    if (joint.name != request->joint_name) {
      continue;
    }
    if (!last_valid_[i]) {
      response->success = false;
      response->message = "no valid encoder reading for " + joint.name;
      return;
    }
    if (request->set_direction) {
      if (request->direction_value != 1 && request->direction_value != -1) {
        response->success = false;
        response->message = "direction must be +1 or -1";
        return;
      }
      joint.direction = request->direction_value;
    }

    // Choose the offset that makes the current pose read `known_position`.
    const double raw = static_cast<double>(last_counts_[i]) *
      static_cast<double>(joint.encoder_direction) / joint.counts_per_joint_radian();
    joint.zero_offset = raw - static_cast<double>(joint.direction) * request->known_position;

    hw_positions_[i] = joint.counts_to_position(last_counts_[i]);
    hw_position_commands_[i] = hw_positions_[i];

    response->success = true;
    response->zero_offset = joint.zero_offset;
    response->message = joint.name + " zeroed at " +
      std::to_string(request->known_position) + " rad";
    RCLCPP_INFO(
      logger_, "Calibrated %s: zero_offset = %.6f rad, direction = %d",
      joint.name.c_str(), joint.zero_offset, joint.direction);

    // The safety layer holds its own copy of the limits and offsets.
    safety_.configure(joint_configs_, safety_limits_);
    return;
  }

  response->success = false;
  response->message = "unknown joint '" + request->joint_name + "'";
}

void RobotArmSystemHardware::handle_save_calibration(
  const std::shared_ptr<robot_arm_interfaces::srv::SaveCalibration::Request> request,
  std::shared_ptr<robot_arm_interfaces::srv::SaveCalibration::Response> response)
{
  const std::string path = request->file_path.empty() ? calibration_file_ : request->file_path;
  if (path.empty()) {
    response->success = false;
    response->message =
      "no file path given and no `calibration_file` parameter set on the hardware";
    return;
  }

  std::string error;
  response->success = write_calibration_file(path, error);
  response->file_path = path;
  response->message = response->success ? "calibration written to " + path : error;
  if (response->success) {
    RCLCPP_INFO(logger_, "Calibration written to %s", path.c_str());
  } else {
    RCLCPP_ERROR(logger_, "Cannot write the calibration: %s", error.c_str());
  }
}

void RobotArmSystemHardware::handle_get_calibration(
  const std::shared_ptr<robot_arm_interfaces::srv::GetCalibration::Request> /*request*/,
  std::shared_ptr<robot_arm_interfaces::srv::GetCalibration::Response> response)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  for (const auto & joint : joint_configs_) {
    robot_arm_msgs::msg::JointCalibration record;
    record.name = joint.name;
    record.zero_offset = joint.zero_offset;
    record.direction = static_cast<int8_t>(joint.direction);
    record.min_position = joint.min_position;
    record.max_position = joint.max_position;
    record.home_position = joint.home_position;
    response->joints.push_back(record);
  }
  response->source_file = calibration_file_;
}

}  // namespace robot_arm_hardware

PLUGINLIB_EXPORT_CLASS(
  robot_arm_hardware::RobotArmSystemHardware, hardware_interface::SystemInterface)
