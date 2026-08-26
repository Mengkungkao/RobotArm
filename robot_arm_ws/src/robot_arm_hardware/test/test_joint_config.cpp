// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
//
// Encoder / calibration conversion tests.  These numbers are what eventually
// move a physical joint, so every path gets a round trip and a sign check.
#include <cmath>
#include <string>
#include <unordered_map>

#include "gtest/gtest.h"
#include "robot_arm_hardware/joint_config.hpp"
#include "robot_arm_hardware/parameter_utils.hpp"

using robot_arm_hardware::JointConfig;
using robot_arm_hardware::joint_config_from_parameters;

namespace
{
/// One encoder count expressed in joint radians - the finest resolution any
/// round trip through integer counts can possibly have.
double one_count(const JointConfig & joint)
{
  return 1.0 / joint.counts_per_joint_radian();
}

JointConfig make_joint(int64_t resolution = 4096, double gear = 100.0)
{
  JointConfig joint;
  joint.name = "joint_1";
  joint.motor_id = 1;
  joint.encoder_resolution = resolution;
  joint.gear_ratio = gear;
  joint.encoder_direction = 1;
  joint.direction = 1;
  joint.zero_offset = 0.0;
  joint.torque_constant = 0.1;
  joint.min_position = -3.1416;
  joint.max_position = 3.1416;
  joint.max_velocity = 2.0;
  joint.max_effort = 100.0;
  return joint;
}
}  // namespace

TEST(JointConfig, CountsPerRadianFollowsResolutionAndGearRatio)
{
  const auto joint = make_joint(4096, 100.0);
  EXPECT_NEAR(joint.counts_per_joint_radian(), 4096.0 * 100.0 / (2.0 * M_PI), 1e-9);

  // A different joint on the same arm may have completely different hardware.
  const auto other = make_joint(1024, 30.0);
  EXPECT_NEAR(other.counts_per_joint_radian(), 1024.0 * 30.0 / (2.0 * M_PI), 1e-9);
}

TEST(JointConfig, FullRevolutionOfTheJointIsGearRatioMotorTurns)
{
  const auto joint = make_joint(4096, 100.0);
  EXPECT_EQ(joint.position_to_counts(2.0 * M_PI), 4096 * 100);
}

TEST(JointConfig, PositionRoundTrip)
{
  const auto joint = make_joint();
  for (const double angle : {-3.0, -1.2345, 0.0, 0.5, 1.5708, 3.0}) {
    const int64_t counts = joint.position_to_counts(angle);
    EXPECT_NEAR(joint.counts_to_position(counts), angle, one_count(joint))
      << "angle " << angle;
  }
}

TEST(JointConfig, ZeroOffsetShiftsTheReading)
{
  auto joint = make_joint();
  joint.zero_offset = 0.25;
  // The encoder reads 0.25 rad when the joint is mechanically at zero.
  const int64_t counts_at_zero = joint.position_to_counts(0.0);
  EXPECT_NEAR(joint.counts_to_position(counts_at_zero), 0.0, one_count(joint));
  EXPECT_GT(counts_at_zero, 0);
}

TEST(JointConfig, DirectionInvertsTheJointButNotTheEncoder)
{
  auto joint = make_joint();
  joint.direction = -1;
  const int64_t counts = joint.position_to_counts(1.0);
  EXPECT_LT(counts, 0);
  EXPECT_NEAR(joint.counts_to_position(counts), 1.0, one_count(joint));
}

TEST(JointConfig, EncoderDirectionIsIndependentOfJointDirection)
{
  auto joint = make_joint();
  joint.encoder_direction = -1;
  const int64_t counts = joint.position_to_counts(1.0);
  EXPECT_LT(counts, 0);
  EXPECT_NEAR(joint.counts_to_position(counts), 1.0, one_count(joint));

  joint.direction = -1;   // both inverted -> back to a positive count
  EXPECT_GT(joint.position_to_counts(1.0), 0);
}

TEST(JointConfig, VelocityRoundTrip)
{
  const auto joint = make_joint();
  const double counts_per_s = joint.velocity_to_counts(1.5);
  EXPECT_NEAR(joint.counts_to_velocity(counts_per_s), 1.5, 1e-9);
}

TEST(JointConfig, EffortUsesTorqueConstantAndReduction)
{
  auto joint = make_joint(4096, 100.0);
  joint.torque_constant = 0.05;
  // 2 A at 0.05 Nm/A through a 100:1 gearbox -> 10 Nm at the joint.
  EXPECT_NEAR(joint.current_to_effort(2.0), 10.0, 1e-9);
  EXPECT_NEAR(joint.effort_to_current(10.0), 2.0, 1e-9);
}

TEST(JointConfig, EffortIsZeroWhenTheDriveCannotReportTorque)
{
  auto joint = make_joint();
  joint.torque_constant = 0.0;
  EXPECT_EQ(joint.effort_to_current(5.0), 0.0);
}

TEST(JointConfig, GearRatioCanBeDelegatedToATransmission)
{
  auto joint = make_joint(4096, 100.0);
  joint.apply_gear_ratio = false;
  EXPECT_NEAR(joint.counts_per_joint_radian(), 4096.0 / (2.0 * M_PI), 1e-9);
}

TEST(JointConfig, ImplausibleCountsAreRejected)
{
  const auto joint = make_joint();
  EXPECT_TRUE(joint.is_plausible_counts(joint.position_to_counts(0.0)));
  EXPECT_TRUE(joint.is_plausible_counts(joint.position_to_counts(3.0)));
  // A stuck bus that returns 0x7FFFFFFF must not be taken for a pose.
  EXPECT_FALSE(joint.is_plausible_counts(2147483647L));
  EXPECT_FALSE(joint.is_plausible_counts(-2147483647L));
}

TEST(JointConfig, ValidateRejectsImpossibleConfigurations)
{
  auto joint = make_joint();
  joint.encoder_resolution = 0;
  EXPECT_THROW(joint.validate(), std::invalid_argument);

  joint = make_joint();
  joint.gear_ratio = 0.0;
  EXPECT_THROW(joint.validate(), std::invalid_argument);

  joint = make_joint();
  joint.direction = 0;
  EXPECT_THROW(joint.validate(), std::invalid_argument);

  joint = make_joint();
  joint.min_position = 1.0;
  joint.max_position = -1.0;
  EXPECT_THROW(joint.validate(), std::invalid_argument);

  joint = make_joint();
  EXPECT_NO_THROW(joint.validate());
}

TEST(JointConfig, ParsedFromRos2ControlParameters)
{
  const std::unordered_map<std::string, std::string> parameters{
    {"motor_id", "3"},
    {"encoder_resolution", "2048"},
    {"gear_ratio", "100.0"},
    {"encoder_direction", "-1"},
    {"torque_constant", "0.07"},
    {"max_current", "6.0"},
    {"max_temperature", "70.0"},
    {"zero_offset", "0.1"},
    {"direction", "1"},
    {"home_position", "0.9"},
    {"min_position", "-2.618"},
    {"max_position", "2.618"},
    {"max_velocity", "2.6"},
    {"max_effort", "100.0"},
  };

  const auto joint = joint_config_from_parameters("joint_3", parameters);
  EXPECT_EQ(joint.name, "joint_3");
  EXPECT_EQ(joint.motor_id, 3);
  EXPECT_EQ(joint.encoder_resolution, 2048);
  EXPECT_EQ(joint.encoder_direction, -1);
  EXPECT_NEAR(joint.zero_offset, 0.1, 1e-9);
  EXPECT_NEAR(joint.home_position, 0.9, 1e-9);
  EXPECT_NEAR(joint.max_position, 2.618, 1e-9);
}

TEST(JointConfig, MalformedParametersAreRejectedNotDefaulted)
{
  std::unordered_map<std::string, std::string> parameters{
    {"motor_id", "1"}, {"encoder_resolution", "4096"}, {"gear_ratio", "one hundred"},
    {"min_position", "-1"}, {"max_position", "1"},
    {"max_velocity", "2"}, {"max_effort", "50"}};
  EXPECT_THROW(joint_config_from_parameters("joint_1", parameters), std::invalid_argument);
}

TEST(JointConfig, ParametersThatScaleTheMotionAreMandatory)
{
  // A defaulted gear ratio or joint limit would move a real machine by the
  // wrong amount, or past its stops, so a missing one must be an error.
  const std::unordered_map<std::string, std::string> complete{
    {"motor_id", "1"}, {"encoder_resolution", "4096"}, {"gear_ratio", "100.0"},
    {"min_position", "-1.0"}, {"max_position", "1.0"},
    {"max_velocity", "2.0"}, {"max_effort", "50.0"}};
  EXPECT_NO_THROW(joint_config_from_parameters("joint_1", complete));

  for (const std::string required :
    {"motor_id", "encoder_resolution", "gear_ratio", "min_position", "max_position",
      "max_velocity", "max_effort"})
  {
    auto incomplete = complete;
    incomplete.erase(required);
    EXPECT_THROW(joint_config_from_parameters("joint_1", incomplete), std::invalid_argument)
      << "missing '" << required << "' was silently defaulted";
  }
}

TEST(JointConfig, CalibrationAndThermalValuesHaveSafeDefaults)
{
  // An unstated encoder sign is "as wired", an unstated offset is
  // "uncalibrated": both are correct behaviour, not a mis-scaled motion.
  const std::unordered_map<std::string, std::string> minimal{
    {"motor_id", "2"}, {"encoder_resolution", "2048"}, {"gear_ratio", "50.0"},
    {"min_position", "-2.0"}, {"max_position", "2.0"},
    {"max_velocity", "1.0"}, {"max_effort", "20.0"}};

  const auto joint = joint_config_from_parameters("joint_2", minimal);
  EXPECT_EQ(joint.encoder_direction, 1);
  EXPECT_EQ(joint.direction, 1);
  EXPECT_DOUBLE_EQ(joint.zero_offset, 0.0);
  EXPECT_DOUBLE_EQ(joint.home_position, 0.0);
  EXPECT_GT(joint.max_temperature, 0.0);
}

TEST(JointConfig, XacroStyleBooleansAreAccepted)
{
  // Xacro serialises Python booleans as "True"/"False".
  const std::unordered_map<std::string, std::string> parameters{{"flag", "True"}};
  EXPECT_TRUE(robot_arm_hardware::get_bool(parameters, "flag", false));
  EXPECT_TRUE(robot_arm_hardware::parse_bool("true", "flag"));
  EXPECT_FALSE(robot_arm_hardware::parse_bool("False", "flag"));
  EXPECT_THROW(robot_arm_hardware::parse_bool("maybe", "flag"), std::invalid_argument);
}
