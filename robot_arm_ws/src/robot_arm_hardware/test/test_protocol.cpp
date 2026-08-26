// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
//
// Protocol layer: wire format of the reference ASCII protocol and the
// behaviour of the simulated drives used for hardware-less bring-up.
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "robot_arm_hardware/joint_config.hpp"
#include "robot_arm_hardware/protocol/loopback_protocol.hpp"
#include "robot_arm_hardware/protocol/motor_protocol.hpp"
#include "robot_arm_hardware/protocol/simple_ascii_protocol.hpp"
#include "robot_arm_hardware/transport/loopback_transport.hpp"

using robot_arm_hardware::create_protocol;
using robot_arm_hardware::Frame;
using robot_arm_hardware::JointConfig;
using robot_arm_hardware::LoopbackProtocol;
using robot_arm_hardware::LoopbackTransport;
using robot_arm_hardware::MotorCommand;
using robot_arm_hardware::MotorFeedback;
using robot_arm_hardware::ProtocolConfig;
using robot_arm_hardware::SimpleAsciiProtocol;
using robot_arm_hardware::TransportConfig;

namespace
{
std::vector<JointConfig> three_joints()
{
  std::vector<JointConfig> joints;
  for (int i = 0; i < 3; ++i) {
    JointConfig joint;
    joint.name = "joint_" + std::to_string(i + 1);
    joint.motor_id = i + 1;
    joint.encoder_resolution = 4096;
    joint.gear_ratio = 100.0;
    joint.min_position = -3.0;
    joint.max_position = 3.0;
    joint.max_velocity = 2.0;
    joint.max_effort = 50.0;
    joint.torque_constant = 0.05;
    joints.push_back(joint);
  }
  return joints;
}
}  // namespace

TEST(ProtocolFactory, CreatesEveryAdvertisedType)
{
  for (const auto & type : robot_arm_hardware::available_protocols()) {
    ProtocolConfig config;
    config.type = type;
    std::string error;
    EXPECT_NE(create_protocol(config, error), nullptr) << type << ": " << error;
  }
}

TEST(ProtocolFactory, RejectsAnUnknownType)
{
  ProtocolConfig config;
  config.type = "morse";
  std::string error;
  EXPECT_EQ(create_protocol(config, error), nullptr);
  EXPECT_NE(error.find("morse"), std::string::npos);
}

// ---------------------------------------------------------------------------
// simple_ascii
// ---------------------------------------------------------------------------

TEST(SimpleAscii, ChecksumIsAnXorOfTheBody)
{
  EXPECT_EQ(SimpleAsciiProtocol::checksum("Q"), "51");        // 'Q' == 0x51
  EXPECT_EQ(SimpleAsciiProtocol::checksum("E 1"), "54");      // 0x45^0x20^0x31
  EXPECT_EQ(SimpleAsciiProtocol::checksum("").size(), 2u);
}

TEST(SimpleAscii, FramesAndUnframesRoundTrip)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  SimpleAsciiProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  const std::string framed = protocol.frame("Q");
  EXPECT_EQ(framed, "#Q*51");

  std::string body;
  ASSERT_TRUE(protocol.unframe(framed, body, error)) << error;
  EXPECT_EQ(body, "Q");
}

TEST(SimpleAscii, RejectsACorruptChecksum)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  SimpleAsciiProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  std::string body;
  EXPECT_FALSE(protocol.unframe("#Q*FF", body, error));
  EXPECT_NE(error.find("checksum"), std::string::npos);
  EXPECT_FALSE(protocol.unframe("Q*51", body, error));   // missing '#'
}

TEST(SimpleAscii, SendsOneFrameWithEverySetpoint)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  SimpleAsciiProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  std::vector<MotorCommand> commands;
  for (int i = 0; i < 3; ++i) {
    MotorCommand command;
    command.motor_id = i + 1;
    command.target_counts = 1000.0 * (i + 1);
    commands.push_back(command);
  }
  commands[2].velocity_mode = true;
  commands[2].target_counts_per_s = -250.0;

  ASSERT_TRUE(protocol.write_commands(commands, error)) << error;

  Frame frame;
  ASSERT_TRUE(transport.read(frame, error));
  const std::string sent = frame.as_string();
  EXPECT_EQ(sent.rfind("#C ", 0), 0u);
  EXPECT_NE(sent.find("1:P1000"), std::string::npos);
  EXPECT_NE(sent.find("2:P2000"), std::string::npos);
  EXPECT_NE(sent.find("3:V-250"), std::string::npos);

  std::string body;
  EXPECT_TRUE(protocol.unframe(sent, body, error)) << error;   // checksum is valid
}

TEST(SimpleAscii, ParsesFeedbackAndConvertsUnits)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  SimpleAsciiProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  // The reply the firmware would send: counts, counts/s, mA, 0.1 degC, fault.
  const std::string body = "F 1:4096,120,850,310,0 2:-2048,-45,900,305,0 3:0,0,0,300,7";
  transport.inject(Frame::from_string(protocol.frame(body)));

  std::vector<MotorFeedback> feedback;
  ASSERT_TRUE(protocol.read_feedback(feedback, error)) << error;
  ASSERT_EQ(feedback.size(), 3u);

  EXPECT_EQ(feedback[0].counts, 4096);
  EXPECT_NEAR(feedback[0].counts_per_s, 120.0, 1e-9);
  EXPECT_NEAR(feedback[0].current, 0.85, 1e-9);        // mA -> A
  EXPECT_NEAR(feedback[0].temperature, 31.0, 1e-9);    // 0.1 degC -> degC
  EXPECT_TRUE(feedback[0].valid);

  EXPECT_EQ(feedback[1].counts, -2048);
  EXPECT_EQ(feedback[2].fault_code, 7);

  // The query frame itself was written to the transport before the reply.
  EXPECT_EQ(feedback[2].motor_id, 3);
}

TEST(SimpleAscii, AMissingAxisIsReportedInvalidNotStale)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  SimpleAsciiProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  transport.inject(Frame::from_string(protocol.frame("F 1:4096,0,0,300,0")));
  std::vector<MotorFeedback> feedback;
  ASSERT_TRUE(protocol.read_feedback(feedback, error));
  ASSERT_EQ(feedback.size(), 3u);
  EXPECT_TRUE(feedback[0].valid);
  EXPECT_FALSE(feedback[1].valid);
  EXPECT_FALSE(feedback[2].valid);
}

TEST(SimpleAscii, RejectsAMalformedReply)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  SimpleAsciiProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  transport.inject(Frame::from_string(protocol.frame("F 1:4096,0,0")));   // too few fields
  std::vector<MotorFeedback> feedback;
  EXPECT_FALSE(protocol.read_feedback(feedback, error));
  EXPECT_FALSE(error.empty());
}

TEST(SimpleAscii, EnableRequiresAnAcknowledgement)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  SimpleAsciiProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  transport.inject(Frame::from_string(protocol.frame("A OK")));
  EXPECT_TRUE(protocol.enable(true, error)) << error;

  transport.inject(Frame::from_string(protocol.frame("A ERR overtemperature")));
  EXPECT_FALSE(protocol.enable(true, error));
  EXPECT_NE(error.find("refused"), std::string::npos);
}

TEST(SimpleAscii, WorksWithoutChecksumsWhenConfiguredSo)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  SimpleAsciiProtocol protocol;
  ProtocolConfig config;
  config.checksum = false;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  EXPECT_EQ(protocol.frame("Q"), "#Q");
  std::string body;
  EXPECT_TRUE(protocol.unframe("#Q", body, error));
  EXPECT_EQ(body, "Q");
}

// ---------------------------------------------------------------------------
// loopback (simulated drives)
// ---------------------------------------------------------------------------

TEST(LoopbackProtocol, StartsAtTheCalibratedZeroAndHoldsWhileDisabled)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  LoopbackProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  std::vector<MotorFeedback> feedback;
  ASSERT_TRUE(protocol.read_feedback(feedback, error));
  ASSERT_EQ(feedback.size(), 3u);
  EXPECT_EQ(feedback[0].counts, 0);
  EXPECT_TRUE(feedback[0].valid);

  // Commanding motion while the drives are disabled must not move anything.
  std::vector<MotorCommand> commands(3);
  for (int i = 0; i < 3; ++i) {
    commands[i].motor_id = i + 1;
    commands[i].target_counts = 50000.0;
  }
  ASSERT_TRUE(protocol.write_commands(commands, error));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_TRUE(protocol.read_feedback(feedback, error));
  EXPECT_EQ(feedback[0].counts, 0);
}

TEST(LoopbackProtocol, TracksTheSetpointOnceEnabled)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  LoopbackProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));
  ASSERT_TRUE(protocol.enable(true, error));

  std::vector<MotorCommand> commands(3);
  for (int i = 0; i < 3; ++i) {
    commands[i].motor_id = i + 1;
    commands[i].target_counts = 20000.0;
  }

  std::vector<MotorFeedback> feedback;
  for (int cycle = 0; cycle < 60; ++cycle) {
    ASSERT_TRUE(protocol.write_commands(commands, error));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_TRUE(protocol.read_feedback(feedback, error));
  }

  EXPECT_GT(feedback[0].counts, 0);
  EXPECT_LE(feedback[0].counts, 20001);
  EXPECT_TRUE(std::isfinite(feedback[0].current));
  EXPECT_GT(feedback[0].temperature, 20.0);
}

TEST(LoopbackProtocol, StopFreezesTheSetpointAtTheCurrentPose)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  LoopbackProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));
  ASSERT_TRUE(protocol.enable(true, error));

  std::vector<MotorCommand> commands(3);
  for (int i = 0; i < 3; ++i) {
    commands[i].motor_id = i + 1;
    commands[i].target_counts = 100000.0;
  }
  ASSERT_TRUE(protocol.write_commands(commands, error));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  std::vector<MotorFeedback> before;
  ASSERT_TRUE(protocol.read_feedback(before, error));
  ASSERT_TRUE(protocol.stop(error));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  std::vector<MotorFeedback> after;
  ASSERT_TRUE(protocol.read_feedback(after, error));
  EXPECT_EQ(before[0].counts, after[0].counts);
}

TEST(LoopbackProtocol, RejectsAWrongNumberOfCommands)
{
  TransportConfig transport_config;
  LoopbackTransport transport(transport_config);
  LoopbackProtocol protocol;
  ProtocolConfig config;
  std::string error;
  ASSERT_TRUE(transport.open(error));
  ASSERT_TRUE(protocol.initialize(&transport, three_joints(), config, error));

  const std::vector<MotorCommand> commands(2);
  EXPECT_FALSE(protocol.write_commands(commands, error));
  EXPECT_NE(error.find("expected 3"), std::string::npos);
}
