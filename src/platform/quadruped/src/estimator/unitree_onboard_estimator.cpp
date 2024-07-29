/*
 * @file sim_dummy_estimator.cpp
 * @date 7/23/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/estimator/unitree_onboard_estimator.hpp"

using namespace unitree::common;
using namespace unitree::robot;

namespace xmotion {
UnitreeOnboardEstimator::UnitreeOnboardEstimator() {
  low_state_sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(
      low_level_state_topic));
  low_state_sub_->InitChannel(
      std::bind(&UnitreeOnboardEstimator::OnLowLevelStateMessageReceived, this,
                std::placeholders::_1),
      1);

  high_state_sub_.reset(
      new ChannelSubscriber<SportModeState>(sport_mode_state_topic));
  high_state_sub_->InitChannel(
      std::bind(&UnitreeOnboardEstimator::OnSportModeStateMessageReceived, this,
                std::placeholders::_1),
      1);
}

void UnitreeOnboardEstimator::Update(
    const QuadrupedModel::SensorData& sensor_data, double dt) {
  // low level state
  LowLevelState low_state;
  {
    std::lock_guard<std::mutex> lock(low_state_mutex_);
    low_state = low_state_;
  }

  x_hat_.q = sensor_data.q;
  x_hat_.q_dot = sensor_data.q_dot;
  x_hat_.quat_base = sensor_data.quaternion;

  // high level state
  SportModeState high_state;
  {
    std::lock_guard<std::mutex> lock(high_state_mutex_);
    high_state = high_state_;
  }
  for (int i = 0; i < 3; ++i) {
    x_hat_.p_base[i] = high_state.position()[i];
    x_hat_.v_base[i] = high_state.velocity()[i];
  }
}

QuadrupedModel::AllJointVar UnitreeOnboardEstimator::GetEstimatedJointPosition()
    const {
  std::lock_guard<std::mutex> lock(x_hat_mutex_);
  return x_hat_.q;
}

QuadrupedModel::AllJointVar UnitreeOnboardEstimator::GetEstimatedJointVelocity()
    const {
  std::lock_guard<std::mutex> lock(x_hat_mutex_);
  return x_hat_.q_dot;
}

Position3d UnitreeOnboardEstimator::GetEstimatedBasePosition() const {
  std::lock_guard<std::mutex> lock(x_hat_mutex_);
  return x_hat_.p_base;
}

Velocity3d UnitreeOnboardEstimator::GetEstimatedBaseVelocity() const {
  std::lock_guard<std::mutex> lock(x_hat_mutex_);
  return x_hat_.v_base;
}

Quaterniond UnitreeOnboardEstimator::GetEstimatedBaseOrientation() const {
  std::lock_guard<std::mutex> lock(x_hat_mutex_);
  return x_hat_.quat_base;
}

void UnitreeOnboardEstimator::OnLowLevelStateMessageReceived(
    const void* message) {
  auto msg_ptr = static_cast<const unitree_go::msg::dds_::LowState_*>(message);
  std::lock_guard<std::mutex> lock(low_state_mutex_);
  low_state_ = *msg_ptr;
}

void UnitreeOnboardEstimator::OnSportModeStateMessageReceived(
    const void* message) {
  auto msg_ptr =
      static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
  std::lock_guard<std::mutex> lock(high_state_mutex_);
  high_state_ = *msg_ptr;
}

}  // namespace xmotion