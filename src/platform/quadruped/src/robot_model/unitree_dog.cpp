/*
 * unitree_dog.cpp
 *
 * Created on 7/6/24 8:51 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/robot_model/unitree_dog.hpp"

#include "logging/xlogger.hpp"

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

////////////////////////////////////////////////////////////////////////////////

UnitreeDog::UnitreeDog(uint32_t domain_id, const std::string& network_interface,
                       const UnitreeModelProfile& profile)
    : network_interface_(network_interface), profile_(profile) {
  for (int i = 0; i < 4; i++) {
    auto index = static_cast<LegIndex>(i);
    legs_[index] = UnitreeLeg{profile_, index};
  }
  body_to_leg_offsets_ = profile_.body_to_leg_offsets;

  // initialize variables
  InitCommand();

  // initialize publisher/subscriber
  ChannelFactory::Instance()->Init(domain_id, network_interface_);
  cmd_pub_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(
      low_level_cmd_topic));
  cmd_pub_->InitChannel();

  state_sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(
      low_level_state_topic));
  state_sub_->InitChannel(std::bind(&UnitreeDog::OnLowLevelStateMessageReceived,
                                    this, std::placeholders::_1),
                          1);
}

void UnitreeDog::SetFootContactState(const Eigen::Vector4d& contact_state) {
  std::lock_guard<std::mutex> lock(foot_contact_state_mutex_);
  foot_contact_state_ = contact_state;
}

Eigen::Vector4d UnitreeDog::GetFootContactState() const {
  std::lock_guard<std::mutex> lock(foot_contact_state_mutex_);
  return foot_contact_state_;
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

void UnitreeDog::SetJointGains(const AllJointGains& gains) {
  for (int i = 0; i < 4; i++) {
    auto index = static_cast<LegIndex>(i);
    legs_[index].SetJointGains(
        std::array<double, 3>{gains.kp[i * 3], gains.kp[i * 3 + 1],
                              gains.kp[i * 3 + 2]},
        std::array<double, 3>{gains.kd[i * 3], gains.kd[i * 3 + 1],
                              gains.kd[i * 3 + 2]});
  }
}

void UnitreeDog::SetJointCommand(const Command& cmd) {
  // set target state
  for (int i = 0; i < 4; i++) {
    auto index = static_cast<LegIndex>(i);
    legs_[index].SetJointTarget(cmd.q.segment<3>(i * 3),
                                cmd.q_dot.segment<3>(i * 3),
                                cmd.tau.segment<3>(i * 3));
  }
}

void UnitreeDog::ConnectSensorDataQueue(
    std::shared_ptr<DataQueue<SensorData>> queue) {
  sensor_data_queue_ = queue;
}

Position3d UnitreeDog::GetCogOffset() const {
  return profile_.center_of_gravity_offset;
}

double UnitreeDog::GetMass() const { return profile_.mass; }

Eigen::Matrix3d UnitreeDog::GetMomentOfInertia() const {
  return profile_.moment_of_inertia;
}

Position3d UnitreeDog::GetFootPosition(LegIndex leg_index,
                                       const JointPosition3d& q,
                                       RefFrame frame) const {
  auto pos = legs_.at(leg_index).GetFootPosition(q);
  if (frame == RefFrame::kBase) {
    pos = pos + body_to_leg_offsets_.at(leg_index);
  }
  return pos;
}

Velocity3d UnitreeDog::GetFootVelocity(LegIndex leg_index,
                                       const JointPosition3d& q,
                                       const JointVelocity3d& q_dot) const {
  return legs_.at(leg_index).GetFootVelocity(q, q_dot);
}

JointPosition3d UnitreeDog::GetJointPosition(LegIndex leg_index,
                                             const Position3d& pos) const {
  return legs_.at(leg_index).GetJointPosition(pos);
}

JointVelocity3d UnitreeDog::GetJointVelocity(LegIndex leg_index,
                                             const Position3d& pos,
                                             const Velocity3d& vel) const {
  return legs_.at(leg_index).GetJointVelocity(pos, vel);
}

JointVelocity3d UnitreeDog::GetJointVelocityQ(LegIndex leg_index,
                                              const JointPosition3d& q,
                                              const Velocity3d& vel) const {
  return legs_.at(leg_index).GetJointVelocityQ(q, vel);
}

UnitreeDog::AllJointVar UnitreeDog::GetJointPosition(
    const std::array<Position3d, 4>& foot_pos, RefFrame frame) const {
  AllJointVar q = AllJointVar::Zero();
  for (int i = 0; i < 4; ++i) {
    auto index = static_cast<LegIndex>(i);
    if (frame == RefFrame::kBase)
      q.segment<3>(i * 3) = legs_.at(index).GetJointPosition(
          foot_pos[i] - body_to_leg_offsets_.at(index));
    else
      q.segment<3>(i * 3) = legs_.at(index).GetJointPosition(foot_pos[i]);
  }
  return q;
}

UnitreeDog::AllJointVar UnitreeDog::GetJointVelocity(
    const std::array<Position3d, 4>& foot_pos,
    const std::array<Velocity3d, 4>& foot_vel, RefFrame frame) const {
  AllJointVar q_dot = AllJointVar::Zero();
  for (int i = 0; i < 4; ++i) {
    auto index = static_cast<LegIndex>(i);
    if (frame == RefFrame::kBase) {
      q_dot.segment<3>(i * 3) = legs_.at(index).GetJointVelocity(
          foot_pos[i] - body_to_leg_offsets_.at(index), foot_vel[i]);
    } else if (frame == RefFrame::kLeg) {
      q_dot.segment<3>(i * 3) =
          legs_.at(index).GetJointVelocity(foot_pos[i], foot_vel[i]);
    }
  }
  return q_dot;
}

std::array<Position3d, 4> UnitreeDog::GetFootPosition(const AllJointVar& q,
                                                      RefFrame frame) const {
  std::array<Position3d, 4> foot_pos;
  for (int i = 0; i < 4; ++i) {
    auto index = static_cast<LegIndex>(i);
    foot_pos[i] = GetFootPosition(index, q.segment<3>(i * 3), frame);
  }
  return foot_pos;
}

std::array<Velocity3d, 4> UnitreeDog::GetFootVelocity(
    const AllJointVar& q, const AllJointVar& q_dot) const {
  std::array<Velocity3d, 4> foot_vel;
  for (int i = 0; i < 4; ++i) {
    auto index = static_cast<LegIndex>(i);
    foot_vel[i] =
        GetFootVelocity(index, q.segment<3>(i * 3), q_dot.segment<3>(i * 3));
  }
  return foot_vel;
}

Torque3d UnitreeDog::GetJointTorque(LegIndex leg_index, const Position3d& pos,
                                    const Force3d& f) const {
  return legs_.at(leg_index).GetJointTorque(pos, f);
}

Torque3d UnitreeDog::GetJointTorqueQ(LegIndex leg_index,
                                     const JointPosition3d& q,
                                     const Force3d& f) const {
  return legs_.at(leg_index).GetJointTorqueQ(q, f);
}

UnitreeDog::AllJointVar UnitreeDog::GetJointTorqueQ(
    const AllJointVar& q, const std::array<Force3d, 4>& foot_force) const {
  AllJointVar tau = AllJointVar::Zero();
  for (int i = 0; i < 4; ++i) {
    auto index = static_cast<LegIndex>(i);
    tau.segment<3>(i * 3) =
        legs_.at(index).GetJointTorqueQ(q.segment<3>(i * 3), foot_force[i]);
  }
  return tau;
}

void UnitreeDog::SendCommandToRobot() {
  // update motor command
  for (int i = 0; i < 4; i++) {
    auto msgs = legs_[static_cast<LegIndex>(i)].GetMotorCommandMsgs();
    cmd_.motor_cmd()[i * 3] = msgs[0];
    cmd_.motor_cmd()[i * 3 + 1] = msgs[1];
    cmd_.motor_cmd()[i * 3 + 2] = msgs[2];
    //    if (i == 0) {
    //      std::cout << "Motor command of leg " << i << " : " << msgs[0].q() <<
    //      " "
    //                << msgs[1].q() << " " << msgs[2].q() << std::endl;
    //    }
  }

  // calculate crc32
  cmd_.crc() = CalculateCrc32(
      (uint32_t*)&cmd_, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);

  // send command
  cmd_pub_->Write(cmd_);
}

void UnitreeDog::OnLowLevelStateMessageReceived(const void* message) {
  auto msg_ptr = static_cast<const unitree_go::msg::dds_::LowState_*>(message);
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = *msg_ptr;
  // copy state to sensor data queue
  if (sensor_data_queue_ != nullptr) {
    SensorData data;
    data.q = Eigen::Matrix<double, 12, 1>::Zero();
    for (int i = 0; i < 12; ++i) {
      auto& motor_state = state_.motor_state()[i];
      data.q[i] = motor_state.q();
      data.q_dot[i] = motor_state.dq();
    }
    data.quaternion = Quaterniond(
        state_.imu_state().quaternion()[0], state_.imu_state().quaternion()[1],
        state_.imu_state().quaternion()[2], state_.imu_state().quaternion()[3]);
    data.gyroscope = Eigen::Vector3d(state_.imu_state().gyroscope()[0],
                                     state_.imu_state().gyroscope()[1],
                                     state_.imu_state().gyroscope()[2]);
    data.accelerometer = Eigen::Vector3d(state_.imu_state().accelerometer()[0],
                                         state_.imu_state().accelerometer()[1],
                                         state_.imu_state().accelerometer()[2]);
    sensor_data_queue_->Push(data);
  }
}
}  // namespace xmotion