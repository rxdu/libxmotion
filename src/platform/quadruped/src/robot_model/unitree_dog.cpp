/*
 * unitree_dog.cpp
 *
 * Created on 7/6/24 8:51 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/robot_model/unitree_dog.hpp"

using namespace unitree::common;
using namespace unitree::robot;

namespace xmotion {
namespace {
uint32_t CalculateCrc32(uint32_t* ptr, uint32_t len) {
  unsigned int xbit = 0;
  unsigned int data = 0;
  unsigned int CRC32 = 0xFFFFFFFF;
  const unsigned int dwPolynomial = 0x04c11db7;

  for (unsigned int i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (unsigned int bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }

      if (data & xbit) CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }

  return CRC32;
}
}  // namespace

UnitreeDog::UnitreeDog(const UnitreeModelProfile& profile) : profile_(profile) {
  // initialize command
  cmd_.head()[0] = 0xFE;
  cmd_.head()[1] = 0xEF;
  cmd_.level_flag() = 0xFF;
  cmd_.gpio() = 0;

  for (int i = 0; i < 12; i++) {
    cmd_.motor_cmd()[i].mode() = (0x01);  // motor switch to servo (PMSM) mode
//    cmd_.motor_cmd()[i].q() = (PosStopF);
//    cmd_.motor_cmd()[i].kp() = (0);
//    cmd_.motor_cmd()[i].dq() = (VelStopF);
//    cmd_.motor_cmd()[i].kd() = (0);
//    cmd_.motor_cmd()[i].tau() = (0);
  }

  /*create publisher*/
  cmd_pub_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(
      low_level_cmd_topic));
  cmd_pub_->InitChannel();

  /*create subscriber*/
  state_sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(
      low_level_state_topic));
  state_sub_->InitChannel(std::bind(&UnitreeDog::OnLowLevelStateMessageReceived,
                                    this, std::placeholders::_1),
                          1);
}

void UnitreeDog::SendCommandToRobot() {}

void UnitreeDog::OnLowLevelStateMessageReceived(const void* message) {
  auto msg_ptr = static_cast<const unitree_go::msg::dds_::LowState_*>(message);
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = *msg_ptr;
}
}  // namespace xmotion