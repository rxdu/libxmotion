/* 
 * vesc_cmd_packet.cpp
 *
 * Created on 7/1/22 9:04 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "motor_vesc/vesc_cmd_packet.hpp"

#include <iostream>

namespace robosw {
namespace {
void ClampCommand(float &cmd, float min, float max) {
  if (cmd > max) cmd = max;
  if (cmd < min) cmd = min;
}
}

VescSetServoPosCmdPacket::VescSetServoPosCmdPacket(uint8_t vesc_id, float pos) {
  // limit command
  ClampCommand(pos, 0.0f, 1.0f);

  // convert to can frame
  frame_.can_id = VescFrame::VescProcessShortBufferCmdFrameId | vesc_id | CAN_EFF_FLAG;
  frame_.can_dlc = 5;
  frame_.data[0] = vesc_id;
  frame_.data[1] = 0x00; // command type: send
  frame_.data[2] = VescFrame::VescCommSetServoPosId;

  int16_t pos_cmd = static_cast<int16_t>(pos * 1000);
  frame_.data[3] = (static_cast<uint16_t>(pos_cmd) & 0xff00) >> 8;
  frame_.data[4] = static_cast<uint16_t>(pos_cmd) & 0x00ff;
}

VescSetDutyCycleCmdPacket::VescSetDutyCycleCmdPacket(uint8_t vesc_id, float duty) {
  // limit command
  ClampCommand(duty, 0.0f, 1.0f);

  // convert to can frame
  frame_.can_id = VescFrame::VescDutyCycleCmdFrameId | vesc_id | CAN_EFF_FLAG;
  frame_.can_dlc = 4;
  int32_t duty_cmd = static_cast<int32_t>(duty * 100000.0f);
  frame_.data[0] = (duty_cmd & 0xff000000) >> 24;
  frame_.data[1] = (duty_cmd & 0x00ff0000) >> 16;
  frame_.data[2] = (duty_cmd & 0x0000ff00) >> 8;
  frame_.data[3] = (duty_cmd & 0x000000ff);
}

VescSetCurrentCmdPacket::VescSetCurrentCmdPacket(uint8_t vesc_id, float current) {
  // limit command
  ClampCommand(current, 0.0f, 20.0f);

  // convert to can frame
  frame_.can_id = VescFrame::VescCurrentCmdFrameId | vesc_id | CAN_EFF_FLAG;
  frame_.can_dlc = 4;
  int32_t duty_cmd = static_cast<int32_t>(current * 1000.0f);
  frame_.data[0] = (duty_cmd & 0xff000000) >> 24;
  frame_.data[1] = (duty_cmd & 0x00ff0000) >> 16;
  frame_.data[2] = (duty_cmd & 0x0000ff00) >> 8;
  frame_.data[3] = (duty_cmd & 0x000000ff);
}

VescSetCurrentBrakeCmdPacket::VescSetCurrentBrakeCmdPacket(uint8_t vesc_id, float current) {
  // limit command
  ClampCommand(current, 0.0f, 20.0f);

  // convert to can frame
  frame_.can_id = VescFrame::VescCurrentBrakeCmdFrameId | vesc_id | CAN_EFF_FLAG;
  frame_.can_dlc = 4;
  int32_t duty_cmd = static_cast<int32_t>(current * 1000.0f);
  frame_.data[0] = (duty_cmd & 0xff000000) >> 24;
  frame_.data[1] = (duty_cmd & 0x00ff0000) >> 16;
  frame_.data[2] = (duty_cmd & 0x0000ff00) >> 8;
  frame_.data[3] = (duty_cmd & 0x000000ff);
}

VescSetRpmCmdPacket::VescSetRpmCmdPacket(uint8_t vesc_id, int32_t rpm) {
  // convert to can frame
  frame_.can_id = VescFrame::VescRpmCmdFrameId | vesc_id | CAN_EFF_FLAG;
  frame_.can_dlc = 4;
  frame_.data[0] = (rpm & 0xff000000) >> 24;
  frame_.data[1] = (rpm & 0x00ff0000) >> 16;
  frame_.data[2] = (rpm & 0x0000ff00) >> 8;
  frame_.data[3] = (rpm & 0x000000ff);
}

VescSetPositionCmdPacket::VescSetPositionCmdPacket(uint8_t vesc_id, float pos) {
  // limit command
  ClampCommand(pos, 0.0f, 1.0f);

  // convert to can frame
  frame_.can_id = VescFrame::VescPositionCmdFrameId | vesc_id | CAN_EFF_FLAG;
  frame_.can_dlc = 4;
  int32_t pos_cmd = static_cast<int32_t>(pos * 100000.0f);
  frame_.data[0] = (pos_cmd & 0xff000000) >> 24;
  frame_.data[1] = (pos_cmd & 0x00ff0000) >> 16;
  frame_.data[2] = (pos_cmd & 0x0000ff00) >> 8;
  frame_.data[3] = (pos_cmd & 0x000000ff);
}
}
