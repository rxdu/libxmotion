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
  //  x_hat_.q = Eigen::Matrix<double, 12, 1>::Zero();
  //  x_hat_.q_dot = Eigen::Matrix<double, 12, 1>::Zero();
  //  x_hat_.tau = Eigen::Matrix<double, 12, 1>::Zero();
  for (int i = 0; i < 12; ++i) {
    auto& motor_state = low_state.motor_state()[i];
    //    x_hat_.q[i] = motor_state.q();
    //    x_hat_.q_dot[i] = motor_state.dq();
    x_hat_.tau[i] = motor_state.tau_est();
    x_hat_.q_ddot[i] = motor_state.ddq();
  }
  x_hat_.q = sensor_data.q;
  x_hat_.q_dot = sensor_data.q_dot;

  // high level state
  //  SportModeState state_feedback;
  //  {
  //    std::lock_guard<std::mutex> lock(high_state_mutex_);
  //    state_feedback = high_state_;
  //  }
}

QuadrupedModel::State UnitreeOnboardEstimator::GetCurrentState() const {
  return x_hat_;
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