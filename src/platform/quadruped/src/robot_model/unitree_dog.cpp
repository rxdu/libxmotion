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

UnitreeDog::UnitreeDog(const std::string& network_interface,
                       const UnitreeModelProfile& profile)
    : network_interface_(network_interface), profile_(profile) {
  for (int i = 0; i < 4; i++) {
    auto index = static_cast<LegIndex>(i);
    legs_[index] = UnitreeLeg{profile_, index};
  }

  // initialize variables
  InitCommand();

  // initialize publisher/subscriber
  ChannelFactory::Instance()->Init(0, network_interface_);
  cmd_pub_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(
      low_level_cmd_topic));
  cmd_pub_->InitChannel();

  state_sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(
      low_level_state_topic));
  state_sub_->InitChannel(std::bind(&UnitreeDog::OnLowLevelStateMessageReceived,
                                    this, std::placeholders::_1),
                          1);
}

void UnitreeDog::InitCommand() {
  // initialize command
  cmd_.head()[0] = 0xFE;
  cmd_.head()[1] = 0xEF;
  cmd_.level_flag() = 0xFF;
  cmd_.gpio() = 0;

  for (int i = 0; i < 4; i++) {
    auto index = static_cast<LegIndex>(i);
    legs_[index] = UnitreeLeg(profile_, index);
    legs_[index].Enable();
  }

  for (int i = 0; i < 4; i++) {
    auto msgs = legs_[static_cast<LegIndex>(i)].GetMotorCommandMsgs();
    cmd_.motor_cmd()[i * 3] = msgs[0];
    cmd_.motor_cmd()[i * 3 + 1] = msgs[1];
    cmd_.motor_cmd()[i * 3 + 2] = msgs[2];
  }
}

void UnitreeDog::SetJointGains(const JointGains& gains) {
  for (int i = 0; i < 4; i++) {
    auto index = static_cast<LegIndex>(i);
    legs_[index].SetJointGains(
        std::array<double, 3>{gains.kp[i * 3], gains.kp[i * 3 + 1],
                              gains.kp[i * 3 + 2]},
        std::array<double, 3>{gains.kd[i * 3], gains.kd[i * 3 + 1],
                              gains.kd[i * 3 + 2]});
  }
}

void UnitreeDog::SetTargetState(const State& state) {
  // set target state
  legs_[LegIndex::kFrontRight].SetJointTarget(state.q.segment<3>(0),
                                              state.q_dot.segment<3>(0),
                                              state.tau.segment<3>(0));
  legs_[LegIndex::kFrontLeft].SetJointTarget(state.q.segment<3>(3),
                                             state.q_dot.segment<3>(3),
                                             state.tau.segment<3>(3));
  legs_[LegIndex::kRearRight].SetJointTarget(state.q.segment<3>(6),
                                             state.q_dot.segment<3>(6),
                                             state.tau.segment<3>(6));
  legs_[LegIndex::kRearLeft].SetJointTarget(state.q.segment<3>(9),
                                            state.q_dot.segment<3>(9),
                                            state.tau.segment<3>(9));
}

void UnitreeDog::SendCommandToRobot() {}

void UnitreeDog::OnLowLevelStateMessageReceived(const void* message) {
  auto msg_ptr = static_cast<const unitree_go::msg::dds_::LowState_*>(message);
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = *msg_ptr;
}
}  // namespace xmotion