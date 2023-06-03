/*
 * vesc_cmd_packet.hpp
 *
 * Created on 7/1/22 9:04 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_CMD_PACKET_HPP
#define ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_CMD_PACKET_HPP

#include "motor_vesc/vesc_frame.hpp"

namespace xmotion {
class VescSetServoPosCmdPacket : public VescFrame {
 public:
  VescSetServoPosCmdPacket(uint8_t vesc_id, float pos);
};

class VescSetDutyCycleCmdPacket : public VescFrame {
 public:
  VescSetDutyCycleCmdPacket(uint8_t vesc_id, float duty);
};

class VescSetCurrentCmdPacket : public VescFrame {
 public:
  VescSetCurrentCmdPacket(uint8_t vesc_id, float current);
};

class VescSetCurrentBrakeCmdPacket : public VescFrame {
 public:
  VescSetCurrentBrakeCmdPacket(uint8_t vesc_id, float current);
};

class VescSetRpmCmdPacket : public VescFrame {
 public:
  VescSetRpmCmdPacket(uint8_t vesc_id, int32_t rpm);
};

class VescSetPositionCmdPacket : public VescFrame {
 public:
  VescSetPositionCmdPacket(uint8_t vesc_id, float pos);
};
}  // namespace xmotion

#endif  // ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_CMD_PACKET_HPP
