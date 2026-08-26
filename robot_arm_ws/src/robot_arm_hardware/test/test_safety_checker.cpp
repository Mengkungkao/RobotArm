// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
//
// The safety layer is the last thing between a planner and a physical arm, so
// every rule it implements is pinned down here.
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "robot_arm_hardware/safety_checker.hpp"

using robot_arm_hardware::JointConfig;
using robot_arm_hardware::SafetyChecker;
using robot_arm_hardware::SafetyLimits;
using robot_arm_hardware::SafetyReport;

namespace
{
constexpr double kPeriod = 0.01;   // 100 Hz

std::vector<JointConfig> two_joints()
{
  std::vector<JointConfig> joints;
  for (int i = 0; i < 2; ++i) {
    JointConfig joint;
    joint.name = "joint_" + std::to_string(i + 1);
    joint.motor_id = i + 1;
    joint.encoder_resolution = 4096;
    joint.gear_ratio = 100.0;
    joint.min_position = -1.0;
    joint.max_position = 1.0;
    joint.max_velocity = 2.0;
    joint.max_effort = 50.0;
    joints.push_back(joint);
  }
  return joints;
}

/// SafetyChecker owns a mutex and is therefore neither copyable nor movable:
/// configure it in place.
void configure(SafetyChecker & checker, SafetyLimits limits = SafetyLimits{})
{
  checker.configure(two_joints(), limits);
}
}  // namespace

TEST(SafetyChecker, PassesThroughValidCommands)
{
  SafetyChecker checker;
  configure(checker);
  checker.notify_command(0.0);
  checker.notify_feedback(0.0);

  std::vector<double> positions{0.5, -0.5};
  std::vector<double> velocities{0.1, -0.1};
  const std::vector<double> measured{0.5, -0.5};

  const auto report = checker.check_commands(positions, velocities, measured, kPeriod, 0.0);
  EXPECT_EQ(report.level, SafetyReport::Level::kOk);
  EXPECT_TRUE(report.motion_allowed);
  EXPECT_DOUBLE_EQ(positions[0], 0.5);
}

TEST(SafetyChecker, ClampsPositionCommandsToTheJointLimits)
{
  SafetyChecker checker;
  configure(checker);
  checker.notify_command(0.0);
  checker.notify_feedback(0.0);

  std::vector<double> positions{5.0, -5.0};
  std::vector<double> velocities{0.0, 0.0};
  const std::vector<double> measured{0.9, -0.9};

  const auto report = checker.check_commands(positions, velocities, measured, 10.0, 0.0);
  EXPECT_TRUE(report.position_limit_violation);
  EXPECT_EQ(report.level, SafetyReport::Level::kViolation);
  EXPECT_LE(positions[0], 1.0);
  EXPECT_GE(positions[1], -1.0);
  EXPECT_EQ(report.violating_joints.size(), 2u);
}

TEST(SafetyChecker, HonoursAnExtraPositionMargin)
{
  SafetyLimits limits;
  limits.position_margin = 0.2;
  SafetyChecker checker;
  configure(checker, limits);
  checker.notify_command(0.0);
  checker.notify_feedback(0.0);

  std::vector<double> positions{1.0, 0.0};
  std::vector<double> velocities{0.0, 0.0};
  const std::vector<double> measured{0.0, 0.0};

  checker.check_commands(positions, velocities, measured, 10.0, 0.0);
  EXPECT_NEAR(positions[0], 0.8, 1e-9);
}

TEST(SafetyChecker, LimitsHowFarASetpointMayJumpInOneCycle)
{
  SafetyChecker checker;
  configure(checker);
  checker.notify_command(0.0);
  checker.notify_feedback(0.0);

  // 2 rad/s * 0.01 s = 0.02 rad is the most one cycle may move.
  std::vector<double> positions{0.9, 0.0};
  std::vector<double> velocities{0.0, 0.0};
  const std::vector<double> measured{0.0, 0.0};

  const auto report = checker.check_commands(positions, velocities, measured, kPeriod, 0.0);
  EXPECT_TRUE(report.velocity_limit_violation);
  EXPECT_NEAR(positions[0], 0.02, 1e-9);
}

TEST(SafetyChecker, ClampsVelocityCommands)
{
  SafetyChecker checker;
  configure(checker);
  checker.notify_command(0.0);
  checker.notify_feedback(0.0);

  std::vector<double> positions{0.0, 0.0};
  std::vector<double> velocities{9.0, -9.0};
  const std::vector<double> measured{0.0, 0.0};

  const auto report = checker.check_commands(positions, velocities, measured, kPeriod, 0.0);
  EXPECT_TRUE(report.velocity_limit_violation);
  EXPECT_NEAR(velocities[0], 2.0, 1e-9);
  EXPECT_NEAR(velocities[1], -2.0, 1e-9);
}

TEST(SafetyChecker, VelocityScaleReducesTheAllowedSpeed)
{
  SafetyLimits limits;
  limits.velocity_scale = 0.5;
  SafetyChecker checker;
  configure(checker, limits);
  checker.notify_command(0.0);
  checker.notify_feedback(0.0);

  std::vector<double> positions{0.0, 0.0};
  std::vector<double> velocities{9.0, 0.0};
  const std::vector<double> measured{0.0, 0.0};

  checker.check_commands(positions, velocities, measured, kPeriod, 0.0);
  EXPECT_NEAR(velocities[0], 1.0, 1e-9);
}

TEST(SafetyChecker, NanCommandsNeverReachTheDrives)
{
  SafetyChecker checker;
  configure(checker);
  checker.notify_command(0.0);
  checker.notify_feedback(0.0);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> positions{nan, 0.0};
  std::vector<double> velocities{nan, 0.0};
  const std::vector<double> measured{0.3, 0.0};

  const auto report = checker.check_commands(positions, velocities, measured, kPeriod, 0.0);
  EXPECT_TRUE(std::isfinite(positions[0]));
  EXPECT_NEAR(positions[0], 0.3, 1e-9);   // holds the measured pose
  EXPECT_DOUBLE_EQ(velocities[0], 0.0);
  EXPECT_TRUE(report.position_limit_violation);
}

TEST(SafetyChecker, EmergencyStopBlocksMotionAndHoldsThePose)
{
  SafetyChecker checker;
  configure(checker);
  checker.notify_command(0.0);
  checker.notify_feedback(0.0);
  checker.set_e_stop(true, "operator");

  std::vector<double> positions{0.9, -0.9};
  std::vector<double> velocities{1.0, -1.0};
  const std::vector<double> measured{0.1, -0.1};

  const auto report = checker.check_commands(positions, velocities, measured, kPeriod, 0.0);
  EXPECT_EQ(report.level, SafetyReport::Level::kEStop);
  EXPECT_TRUE(report.e_stop_active);
  EXPECT_FALSE(report.motion_allowed);
  EXPECT_NEAR(positions[0], 0.1, 1e-9);
  EXPECT_NEAR(positions[1], -0.1, 1e-9);
  EXPECT_DOUBLE_EQ(velocities[0], 0.0);
  EXPECT_NE(report.message.find("operator"), std::string::npos);

  checker.set_e_stop(false, "");
  EXPECT_FALSE(checker.e_stop_active());
}

TEST(SafetyChecker, StaleCommandsAreNotExecuted)
{
  SafetyLimits limits;
  limits.command_timeout = 0.1;
  SafetyChecker checker;
  configure(checker, limits);
  checker.notify_command(0.0);
  checker.notify_feedback(1.0);   // feedback is fresh, only the command is old

  std::vector<double> positions{0.9, 0.9};
  std::vector<double> velocities{1.0, 1.0};
  const std::vector<double> measured{0.0, 0.0};

  const auto report = checker.check_commands(positions, velocities, measured, kPeriod, 1.0);
  EXPECT_TRUE(report.command_timeout);
  EXPECT_FALSE(report.motion_allowed);
  EXPECT_NEAR(positions[0], 0.0, 1e-9);   // hold, do not continue the old motion
}

TEST(SafetyChecker, LostFeedbackStopsTheArm)
{
  SafetyLimits limits;
  limits.comm_timeout = 0.1;
  SafetyChecker checker;
  configure(checker, limits);
  checker.notify_command(1.0);
  checker.notify_feedback(0.0);

  std::vector<double> positions{0.5, 0.5};
  std::vector<double> velocities{0.5, 0.5};
  const std::vector<double> measured{0.0, 0.0};

  const auto report = checker.check_commands(positions, velocities, measured, kPeriod, 1.0);
  EXPECT_TRUE(report.communication_timeout);
  EXPECT_FALSE(report.motion_allowed);
  EXPECT_DOUBLE_EQ(velocities[0], 0.0);
}

TEST(SafetyChecker, CountsConsecutiveErrors)
{
  SafetyChecker checker;
  configure(checker);
  EXPECT_EQ(checker.consecutive_errors(), 0);
  checker.notify_error();
  checker.notify_error();
  EXPECT_EQ(checker.consecutive_errors(), 2);
  checker.notify_success();
  EXPECT_EQ(checker.consecutive_errors(), 0);
}

TEST(SafetyChecker, RejectsImplausibleAndNonFiniteFeedback)
{
  SafetyChecker checker;
  configure(checker);
  SafetyReport report;

  const std::vector<double> good_positions{0.5, -0.5};
  const std::vector<double> good_velocities{0.0, 0.0};
  EXPECT_TRUE(checker.validate_feedback(good_positions, good_velocities, report));

  SafetyReport bad_report;
  const std::vector<double> far_away{50.0, 0.0};
  EXPECT_FALSE(checker.validate_feedback(far_away, good_velocities, bad_report));
  EXPECT_TRUE(bad_report.invalid_feedback);
  EXPECT_FALSE(bad_report.motion_allowed);

  SafetyReport nan_report;
  const std::vector<double> nan_positions{std::numeric_limits<double>::quiet_NaN(), 0.0};
  EXPECT_FALSE(checker.validate_feedback(nan_positions, good_velocities, nan_report));
  EXPECT_TRUE(nan_report.invalid_feedback);
}

TEST(SafetyChecker, WarnsBeforeItClamps)
{
  SafetyChecker checker;
  configure(checker);
  checker.notify_command(0.0);
  checker.notify_feedback(0.0);

  std::vector<double> positions{0.98, 0.0};   // inside the limit, but close
  std::vector<double> velocities{0.0, 0.0};
  const std::vector<double> measured{0.98, 0.0};

  const auto report = checker.check_commands(positions, velocities, measured, kPeriod, 0.0);
  EXPECT_EQ(report.level, SafetyReport::Level::kWarn);
  EXPECT_TRUE(report.motion_allowed);
  EXPECT_DOUBLE_EQ(positions[0], 0.98);
}
