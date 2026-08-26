// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__ROBOT_ARM_HARDWARE_INTERFACE_HPP_
#define ROBOT_ARM_HARDWARE__ROBOT_ARM_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "robot_arm_hardware/joint_config.hpp"
#include "robot_arm_hardware/protocol/motor_protocol.hpp"
#include "robot_arm_hardware/safety_checker.hpp"
#include "robot_arm_hardware/transport/transport.hpp"
#include "robot_arm_interfaces/srv/calibrate_joint.hpp"
#include "robot_arm_interfaces/srv/get_calibration.hpp"
#include "robot_arm_interfaces/srv/save_calibration.hpp"
#include "robot_arm_interfaces/srv/set_e_stop.hpp"
#include "robot_arm_interfaces/srv/set_motor_enable.hpp"
#include "robot_arm_msgs/msg/arm_status.hpp"
#include "robot_arm_msgs/msg/safety_status.hpp"

namespace robot_arm_hardware
{

/// ros2_control System plugin for the physical 6-DOF arm.
///
///     ros2_control
///          |
///     RobotArmSystemHardware      <- this class: units, safety, diagnostics
///          |
///     MotorProtocol               <- wire format
///          |
///     Transport                   <- serial / RS485 / CAN / TCP
///          |
///     motor controller -> motors
///
/// The class exports exactly the same interfaces as the simulated backend
/// (position + velocity commands, position/velocity/effort states), which is
/// what makes `use_sim:=true|false` invisible to MoveIt and to applications.
///
/// Threading: read()/write() run in the controller_manager's real-time loop.
/// A second thread spins a small ROS node for the e-stop topic, the service
/// interface, the diagnostics and the watchdog.  Everything shared between the
/// two is guarded by `data_mutex_`, and the transport/protocol implementations
/// are individually locked as well.
class RobotArmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotArmSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class ControlMode : uint8_t
  {
    kPosition,
    kVelocity,
  };

  // --- configuration -------------------------------------------------------
  bool parse_hardware_parameters(std::string & error);
  bool parse_joint_parameters(std::string & error);

  // --- ROS side ------------------------------------------------------------
  void start_node();
  void stop_node();
  void publish_diagnostics();
  void watchdog();

  void on_e_stop_message(const std_msgs::msg::Bool::SharedPtr message);
  void handle_set_e_stop(
    const std::shared_ptr<robot_arm_interfaces::srv::SetEStop::Request> request,
    std::shared_ptr<robot_arm_interfaces::srv::SetEStop::Response> response);
  void handle_set_motor_enable(
    const std::shared_ptr<robot_arm_interfaces::srv::SetMotorEnable::Request> request,
    std::shared_ptr<robot_arm_interfaces::srv::SetMotorEnable::Response> response);
  void handle_calibrate_joint(
    const std::shared_ptr<robot_arm_interfaces::srv::CalibrateJoint::Request> request,
    std::shared_ptr<robot_arm_interfaces::srv::CalibrateJoint::Response> response);
  void handle_save_calibration(
    const std::shared_ptr<robot_arm_interfaces::srv::SaveCalibration::Request> request,
    std::shared_ptr<robot_arm_interfaces::srv::SaveCalibration::Response> response);
  void handle_get_calibration(
    const std::shared_ptr<robot_arm_interfaces::srv::GetCalibration::Request> request,
    std::shared_ptr<robot_arm_interfaces::srv::GetCalibration::Response> response);

  // --- helpers -------------------------------------------------------------
  bool engage_e_stop(const std::string & reason);
  bool set_motors_enabled(bool enabled, std::string & error);
  bool write_calibration_file(const std::string & path, std::string & error) const;
  robot_arm_msgs::msg::ArmStatus build_status_message() const;

  static double now_seconds();

  rclcpp::Logger logger_{rclcpp::get_logger("RobotArmSystemHardware")};
  /// Throttled logging must not depend on the (possibly simulated) ROS clock.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // --- configuration -------------------------------------------------------
  std::vector<JointConfig> joint_configs_;
  TransportConfig transport_config_;
  ProtocolConfig protocol_config_;
  SafetyLimits safety_limits_;
  bool enable_on_activate_{true};
  double diagnostics_period_{1.0};
  std::string node_namespace_;
  std::string calibration_file_;
  int max_consecutive_errors_{5};

  // --- communication -------------------------------------------------------
  TransportPtr transport_;
  MotorProtocolPtr protocol_;
  SafetyChecker safety_;

  // --- state / command storage (ros2_control handles point into these) ------
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_velocity_commands_;

  // --- data shared with the node thread ------------------------------------
  mutable std::mutex data_mutex_;
  std::vector<int64_t> last_counts_;
  std::vector<double> last_current_;
  std::vector<double> last_temperature_;
  std::vector<uint8_t> last_fault_;
  std::vector<bool> last_valid_;
  std::vector<ControlMode> control_modes_;
  SafetyReport last_report_;

  std::atomic<bool> connected_{false};
  std::atomic<bool> motors_enabled_{false};
  std::atomic<bool> active_{false};
  std::atomic<uint32_t> read_errors_{0};
  std::atomic<uint32_t> write_errors_{0};
  std::atomic<double> last_read_time_{-1.0};
  std::atomic<double> last_write_time_{-1.0};
  std::atomic<double> last_valid_feedback_time_{-1.0};
  /// Set by the watchdog when the control loop stalled; cleared by the first
  /// write() that follows, which also re-seeds the commands from the encoders
  /// so that no stale setpoint is ever executed.
  std::atomic<bool> watchdog_tripped_{false};
  std::atomic<bool> motion_blocked_{false};

  // --- ROS node ------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread node_thread_;
  std::atomic<bool> node_running_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_subscription_;
  rclcpp::Publisher<robot_arm_msgs::msg::ArmStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<robot_arm_msgs::msg::SafetyStatus>::SharedPtr safety_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Service<robot_arm_interfaces::srv::SetEStop>::SharedPtr e_stop_service_;
  rclcpp::Service<robot_arm_interfaces::srv::SetMotorEnable>::SharedPtr enable_service_;
  rclcpp::Service<robot_arm_interfaces::srv::CalibrateJoint>::SharedPtr calibrate_service_;
  rclcpp::Service<robot_arm_interfaces::srv::SaveCalibration>::SharedPtr save_service_;
  rclcpp::Service<robot_arm_interfaces::srv::GetCalibration>::SharedPtr get_calibration_service_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__ROBOT_ARM_HARDWARE_INTERFACE_HPP_
