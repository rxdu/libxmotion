/*
 * utest_polyline.cpp
 *
 * Created on: Nov 19, 2020 21:00
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "gtest/gtest.h"

#include <cstdint>
#include <cstring>

#include "vesc_driver/vesc_packet.hpp"

using namespace robosw;

TEST(VescPacketTest, VescStatus1Packet) {
  struct can_frame frame;
  uint8_t data[] = {0x00, 0x00, 0x0c, 0xf2, 0x00, 0x02, 0x00, 0x64};
  frame.can_dlc = sizeof(data);
  std::memcpy(frame.data, data, frame.can_dlc);

  VescStatus1Packet packet(frame);

  ASSERT_FLOAT_EQ(packet.GetRpm(), 3314);
  ASSERT_FLOAT_EQ(packet.GetCurrent(), 0.2);
  ASSERT_FLOAT_EQ(packet.GetDuty(), 0.1);
}

TEST(VescPacketTest, VescStatus2Packet) {
  struct can_frame frame;
  uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  frame.can_dlc = sizeof(data);
  std::memcpy(frame.data, data, frame.can_dlc);

  VescStatus2Packet packet(frame);

  ASSERT_FLOAT_EQ(packet.GetAmpHours(), 0);
  ASSERT_FLOAT_EQ(packet.GetAmpHoursCharged(), 0);
}

TEST(VescPacketTest, VescStatus3Packet) {
  struct can_frame frame;
  uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  frame.can_dlc = sizeof(data);
  std::memcpy(frame.data, data, frame.can_dlc);

  VescStatus3Packet packet(frame);

  ASSERT_FLOAT_EQ(packet.GetWattHours(), 0);
  ASSERT_FLOAT_EQ(packet.GetWattHoursCharged(), 0);
}

TEST(VescPacketTest, VescStatus4Packet) {
  struct can_frame frame;
  uint8_t data[] = {0x01, 0x75, 0x01, 0xc1, 0x00, 0x00, 0x2c, 0xc6};
  frame.can_dlc = sizeof(data);
  std::memcpy(frame.data, data, frame.can_dlc);

  VescStatus4Packet packet(frame);

  ASSERT_FLOAT_EQ(packet.GetTempFET(), 37.3);
  ASSERT_FLOAT_EQ(packet.GetTempMotor(), 44.9);
  ASSERT_FLOAT_EQ(packet.GetCurrentIn(), 0);
  ASSERT_FLOAT_EQ(packet.GetPidPosNow(), 229.24);
}

TEST(VescPacketTest, VescStatus5Packet) {
  struct can_frame frame;
  uint8_t data[] = {0x00, 0x04, 0xaa, 0xd5, 0x00, 0xc2, 0x00, 0x00};
  frame.can_dlc = sizeof(data);
  std::memcpy(frame.data, data, frame.can_dlc);

  VescStatus5Packet packet(frame);

  ASSERT_EQ(packet.GetTachoValue(), 305877);
  ASSERT_FLOAT_EQ(packet.GetVoltageIn(), 19.4);
}