/*
 * vesc_frame.hpp
 *
 * Created on 7/1/22 9:01 PM
 * Description:
 *
 * Reference:
 * [1] https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can
 * [2] comm/comm_can.c in https://github.com/vedderb/bldc.git
 * [3] datatypes.h in https://github.com/vedderb/bldc.git
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_FRAME_HPP
#define ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_FRAME_HPP

#include <linux/can.h>

#include <cstdint>

namespace robosw {
class VescFrame {
 public:
  // CAN id
  static constexpr uint32_t VescStatus1FrameId = 0x00000900;
  static constexpr uint32_t VescStatus2FrameId = 0x00000e00;
  static constexpr uint32_t VescStatus3FrameId = 0x00000f00;
  static constexpr uint32_t VescStatus4FrameId = 0x00001000;
  static constexpr uint32_t VescStatus5FrameId = 0x00001b00;

  static constexpr uint32_t VescDutyCycleCmdFrameId = 0x00000000;
  static constexpr uint32_t VescCurrentCmdFrameId = 0x00000100;

  static constexpr uint32_t VescCurrentBrakeCmdFrameId = 0x00000200;
  static constexpr uint32_t VescRpmCmdFrameId = 0x00000300;
  static constexpr uint32_t VescPositionCmdFrameId = 0x00000400;

  static constexpr uint32_t VescProcessShortBufferCmdFrameId = 0x00000800;

  // Command id
  static constexpr uint8_t VescCommSetServoPosId = 0x0c;

 public:
  VescFrame() = default;
  VescFrame(const struct can_frame &frame) : frame_(frame) {};
  virtual ~VescFrame() = default;

  struct can_frame GetCanFrame() const { return frame_; }

 protected:
  struct can_frame frame_;
};
}  // namespace robosw

#endif  // ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_FRAME_HPP
