// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
//
// Transport layer: the factory contract and the loopback implementation used
// by CI and by hardware-less bring-up.
#include <string>

#include "gtest/gtest.h"
#include "robot_arm_hardware/transport/loopback_transport.hpp"
#include "robot_arm_hardware/transport/transport.hpp"

using robot_arm_hardware::create_transport;
using robot_arm_hardware::Frame;
using robot_arm_hardware::LoopbackTransport;
using robot_arm_hardware::TransportConfig;

TEST(TransportFactory, CreatesEveryAdvertisedType)
{
  for (const auto & type : robot_arm_hardware::available_transports()) {
    TransportConfig config;
    config.type = type;
    std::string error;
    const auto transport = create_transport(config, error);
    EXPECT_NE(transport, nullptr) << "type " << type << ": " << error;
    EXPECT_TRUE(error.empty());
  }
}

TEST(TransportFactory, RejectsAnUnknownTypeWithAHelpfulMessage)
{
  TransportConfig config;
  config.type = "carrier_pigeon";
  std::string error;
  EXPECT_EQ(create_transport(config, error), nullptr);
  EXPECT_NE(error.find("carrier_pigeon"), std::string::npos);
  EXPECT_NE(error.find("serial"), std::string::npos);   // lists what is available
}

TEST(TransportFactory, Rs485IsASerialPortWithDirectionControl)
{
  TransportConfig config;
  config.type = "rs485";
  config.rs485_rts_toggle = true;
  config.serial_port = "/dev/ttyUSB7";
  std::string error;
  const auto transport = create_transport(config, error);
  ASSERT_NE(transport, nullptr);
  EXPECT_NE(transport->name().find("rs485"), std::string::npos);
  EXPECT_NE(transport->name().find("/dev/ttyUSB7"), std::string::npos);
}

TEST(TransportFactory, OpeningAMissingSerialPortFailsCleanly)
{
  TransportConfig config;
  config.type = "serial";
  config.serial_port = "/dev/does-not-exist-robot-arm";
  std::string error;
  const auto transport = create_transport(config, error);
  ASSERT_NE(transport, nullptr);
  EXPECT_FALSE(transport->open(error));      // no exception, no crash
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(transport->is_open());
}

TEST(LoopbackTransport, RoundTripsFrames)
{
  TransportConfig config;
  LoopbackTransport transport(config);
  std::string error;

  ASSERT_TRUE(transport.open(error));
  EXPECT_TRUE(transport.is_open());

  ASSERT_TRUE(transport.write(Frame::from_string("#Q*51"), error));
  Frame frame;
  ASSERT_TRUE(transport.read(frame, error));
  EXPECT_EQ(frame.as_string(), "#Q*51");

  // Nothing left: a read must fail rather than return a stale frame.
  EXPECT_FALSE(transport.read(frame, error));
  EXPECT_NE(error.find("timeout"), std::string::npos);
}

TEST(LoopbackTransport, RefusesIoWhileClosed)
{
  TransportConfig config;
  LoopbackTransport transport(config);
  std::string error;
  Frame frame;
  EXPECT_FALSE(transport.write(Frame::from_string("x"), error));
  EXPECT_FALSE(transport.read(frame, error));

  ASSERT_TRUE(transport.open(error));
  transport.close();
  EXPECT_FALSE(transport.is_open());
  EXPECT_FALSE(transport.write(Frame::from_string("x"), error));
}

TEST(LoopbackTransport, CanInjectRepliesAndFailures)
{
  TransportConfig config;
  LoopbackTransport transport(config);
  std::string error;
  ASSERT_TRUE(transport.open(error));

  transport.inject(Frame::from_string("#A OK*1D"));
  Frame frame;
  ASSERT_TRUE(transport.read(frame, error));
  EXPECT_EQ(frame.as_string(), "#A OK*1D");

  transport.fail_next(2);
  EXPECT_FALSE(transport.write(Frame::from_string("x"), error));
  EXPECT_FALSE(transport.read(frame, error));
  EXPECT_TRUE(transport.write(Frame::from_string("x"), error));   // failures exhausted
}

TEST(LoopbackTransport, FlushDropsPendingFrames)
{
  TransportConfig config;
  LoopbackTransport transport(config);
  std::string error;
  ASSERT_TRUE(transport.open(error));

  transport.inject(Frame::from_string("stale"));
  transport.flush();
  Frame frame;
  EXPECT_FALSE(transport.read(frame, error));
}
