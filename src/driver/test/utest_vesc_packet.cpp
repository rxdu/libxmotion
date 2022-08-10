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

#include "driver/vesc_driver/vesc_status_packet.hpp"
#include "driver/vesc_driver/vesc_cmd_packet.hpp"

using namespace robosw;

// TODO most tests don't cover out-of-limit checks

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

TEST(VescPacketTest, VescSetServoPosPacket) {
  VescSetServoPosCmdPacket pkt1(0x68, 0.8);
  auto frame1 = pkt1.GetCanFrame();

  ASSERT_EQ(frame1.can_id, 0x00000868 | CAN_EFF_FLAG);
  ASSERT_EQ(frame1.can_dlc, 5);
  ASSERT_EQ(frame1.data[0], 0x68);
  ASSERT_EQ(frame1.data[1], 0x00);
  ASSERT_EQ(frame1.data[2], 0x0c);
  ASSERT_EQ(frame1.data[3], 0x03);
  ASSERT_EQ(frame1.data[4], 0x20);

  VescSetServoPosCmdPacket pkt2(0x68, 0.5);
  auto frame2 = pkt2.GetCanFrame();

  ASSERT_EQ(frame2.can_id, 0x00000868 | CAN_EFF_FLAG);
  ASSERT_EQ(frame2.can_dlc, 5);
  ASSERT_EQ(frame2.data[0], 0x68);
  ASSERT_EQ(frame2.data[1], 0x00);
  ASSERT_EQ(frame2.data[2], 0x0c);
  ASSERT_EQ(frame2.data[3], 0x01);
  ASSERT_EQ(frame2.data[4], 0xf4);

  VescSetServoPosCmdPacket pkt3(0x68, -0.5);
  auto frame3 = pkt3.GetCanFrame();

  ASSERT_EQ(frame3.can_id, 0x00000868 | CAN_EFF_FLAG);
  ASSERT_EQ(frame3.can_dlc, 5);
  ASSERT_EQ(frame3.data[0], 0x68);
  ASSERT_EQ(frame3.data[1], 0x00);
  ASSERT_EQ(frame3.data[2], 0x0c);
  ASSERT_EQ(frame3.data[3], 0x00);
  ASSERT_EQ(frame3.data[4], 0x00);

  VescSetServoPosCmdPacket pkt4(0x68, 1.5);
  auto frame4 = pkt4.GetCanFrame();

  ASSERT_EQ(frame4.can_id, 0x00000868 | CAN_EFF_FLAG);
  ASSERT_EQ(frame4.can_dlc, 5);
  ASSERT_EQ(frame4.data[0], 0x68);
  ASSERT_EQ(frame4.data[1], 0x00);
  ASSERT_EQ(frame4.data[2], 0x0c);
  ASSERT_EQ(frame4.data[3], 0x03);
  ASSERT_EQ(frame4.data[4], 0xe8);
}

TEST(VescPacketTest, VescSetDutyCyclePacket) {
  VescSetDutyCycleCmdPacket pkt1(0x68, 0.1);
  auto frame1 = pkt1.GetCanFrame();

  ASSERT_EQ(frame1.can_id, 0x00000068 | CAN_EFF_FLAG);
  ASSERT_EQ(frame1.can_dlc, 4);
  ASSERT_EQ(frame1.data[0], 0x00);
  ASSERT_EQ(frame1.data[1], 0x00);
  ASSERT_EQ(frame1.data[2], 0x27);
  ASSERT_EQ(frame1.data[3], 0x10);

  VescSetDutyCycleCmdPacket pkt2(0x68, -0.5);
  auto frame2 = pkt2.GetCanFrame();

  ASSERT_EQ(frame2.can_id, 0x00000068 | CAN_EFF_FLAG);
  ASSERT_EQ(frame2.can_dlc, 4);
  ASSERT_EQ(frame2.data[0], 0x00);
  ASSERT_EQ(frame2.data[1], 0x00);
  ASSERT_EQ(frame2.data[2], 0x00);
  ASSERT_EQ(frame2.data[3], 0x00);

  VescSetDutyCycleCmdPacket pkt3(0x68, 1.5);
  auto frame3 = pkt3.GetCanFrame();

  ASSERT_EQ(frame3.can_id, 0x00000068 | CAN_EFF_FLAG);
  ASSERT_EQ(frame3.can_dlc, 4);
  ASSERT_EQ(frame3.data[0], 0x00);
  ASSERT_EQ(frame3.data[1], 0x01);
  ASSERT_EQ(frame3.data[2], 0x86);
  ASSERT_EQ(frame3.data[3], 0xa0);
}

TEST(VescPacketTest, VescSetCurrentPacket) {
  VescSetCurrentCmdPacket pkt1(0x68, 5);
  auto frame1 = pkt1.GetCanFrame();

  ASSERT_EQ(frame1.can_id, 0x00000168 | CAN_EFF_FLAG);
  ASSERT_EQ(frame1.can_dlc, 4);
  ASSERT_EQ(frame1.data[0], 0x00);
  ASSERT_EQ(frame1.data[1], 0x00);
  ASSERT_EQ(frame1.data[2], 0x13);
  ASSERT_EQ(frame1.data[3], 0x88);
}

TEST(VescPacketTest, VescSetCurrentBrakePacket) {
  VescSetCurrentBrakeCmdPacket pkt1(0x68, 5);
  auto frame1 = pkt1.GetCanFrame();

  ASSERT_EQ(frame1.can_id, 0x00000268 | CAN_EFF_FLAG);
  ASSERT_EQ(frame1.can_dlc, 4);
  ASSERT_EQ(frame1.data[0], 0x00);
  ASSERT_EQ(frame1.data[1], 0x00);
  ASSERT_EQ(frame1.data[2], 0x13);
  ASSERT_EQ(frame1.data[3], 0x88);
}

TEST(VescPacketTest, VescSetRpmPacket) {
  VescSetRpmCmdPacket pkt1(0x68, 1000);
  auto frame1 = pkt1.GetCanFrame();

  ASSERT_EQ(frame1.can_id, 0x00000368 | CAN_EFF_FLAG);
  ASSERT_EQ(frame1.can_dlc, 4);
  ASSERT_EQ(frame1.data[0], 0x00);
  ASSERT_EQ(frame1.data[1], 0x00);
  ASSERT_EQ(frame1.data[2], 0x03);
  ASSERT_EQ(frame1.data[3], 0xe8);
}

TEST(VescPacketTest, VescSetPositionPacket) {
  VescSetPositionCmdPacket pkt1(0x68, 0.1);
  auto frame1 = pkt1.GetCanFrame();

  ASSERT_EQ(frame1.can_id, 0x00000468 | CAN_EFF_FLAG);
  ASSERT_EQ(frame1.can_dlc, 4);
  ASSERT_EQ(frame1.data[0], 0x00);
  ASSERT_EQ(frame1.data[1], 0x00);
  ASSERT_EQ(frame1.data[2], 0x27);
  ASSERT_EQ(frame1.data[3], 0x10);
}