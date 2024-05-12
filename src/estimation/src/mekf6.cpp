/*
 * mekf6.cpp
 *
 * Created on 3/31/24 8:16 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "estimation/mekf6.hpp"

namespace xmotion {
void Mekf6::Initialize(const Params& params) {
  state_ = params.init_state;
  state_cov_ = params.init_state_cov;

  observation_cov_ = params.init_observation_cov;
  gyro_bias_ = params.gyro_bias;
  accel_bias_ = params.accel_bias;

  observation_cov_ = ObservationCovariance::Identity();
}

void Mekf6::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel,
                   double dt) {
  Eigen::Quaterniond q_k_1(state_(0), state_(1), state_(2), state_(3));
  Eigen::Quaterniond q_g(0, gyro(0), gyro(1), gyro(2));
//  Eigen::Quaterniond q_k = q_k_1 + q_k_1 * q_g * dt;
}
}  // namespace xmotion
