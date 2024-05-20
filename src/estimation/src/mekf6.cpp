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
void Mekf6::Initialize(const Params &params) {
  x_ = params.init_state;
  P_ = params.init_state_cov;

  Q_ = params.init_process_noise_cov;
  R_ = params.init_observation_noise_cov;

  params_ = params;
  q_hat_ = params.init_quaternion;
}

Eigen::Matrix<double, Mekf6::StateDimension, Mekf6::StateDimension>
Mekf6::GetQMatrix(double dt) {
  Eigen::Matrix<double, StateDimension, StateDimension> Q =
      Eigen::Matrix<double, StateDimension, StateDimension>::Zero();
  Eigen::Matrix<double, 3, 3> m_sigma_omega, m_sigma_f;
  m_sigma_omega << params_.sigma_omega(0) * params_.sigma_omega(0), 0, 0, 0,
      params_.sigma_omega(1) * params_.sigma_omega(1), 0, 0, 0,
      params_.sigma_omega(2) * params_.sigma_omega(2);
  m_sigma_f << params_.sigma_f(0) * params_.sigma_f(0), 0, 0, 0,
      params_.sigma_f(1) * params_.sigma_f(1), 0, 0, 0,
      params_.sigma_f(2) * params_.sigma_f(2);

  Eigen::Matrix<double, 3, 3> m_sigma_beta_omega, m_sigma_beta_f;
  m_sigma_beta_omega << params_.sigma_beta_omega(0) *
                            params_.sigma_beta_omega(0),
      0, 0, 0, params_.sigma_beta_omega(1) * params_.sigma_beta_omega(1), 0, 0,
      0, params_.sigma_beta_omega(2) * params_.sigma_beta_omega(2);
  m_sigma_beta_f << params_.sigma_beta_f(0) * params_.sigma_beta_f(0), 0, 0, 0,
      params_.sigma_beta_f(1) * params_.sigma_beta_f(1), 0, 0, 0,
      params_.sigma_beta_f(2) * params_.sigma_beta_f(2);

  Q.block<3, 3>(0, 0) =
      m_sigma_omega * dt + m_sigma_beta_omega * dt * dt * dt / 3.0f;
  Q.block<3, 3>(0, 9) = -m_sigma_beta_omega * dt * dt / 2.0f;

  Q.block<3, 3>(3, 3) = m_sigma_f * dt + m_sigma_beta_f * dt * dt * dt / 3.0f;
  Q.block<3, 3>(3, 6) =
      m_sigma_beta_f * dt * dt * dt * dt / 8.0 + m_sigma_beta_f * dt * dt / 2.0;
  Q.block<3, 3>(3, 12) = -m_sigma_beta_f * dt * dt / 2.0;

  Q.block<3, 3>(6, 3) =
      m_sigma_f * dt * dt / 2.0f + m_sigma_beta_f * dt * dt * dt * dt / 8.0f;
  Q.block<3, 3>(6, 6) = m_sigma_f * dt * dt * dt / 3.0f +
                        m_sigma_beta_f * dt * dt * dt * dt * dt / 20.0f;
  Q.block<3, 3>(6, 12) = -m_sigma_beta_f * dt * dt * dt / 6.0f;

  Q.block<3, 3>(9, 0) = -m_sigma_beta_omega * dt * dt / 2.0f;
  Q.block<3, 3>(9, 9) = m_sigma_beta_omega * dt * dt / 2.0f;

  Q.block<3, 3>(12, 3) = -m_sigma_beta_f * dt * dt / 2.0f;
  Q.block<3, 3>(12, 6) = -m_sigma_beta_f * dt * dt * dt / 6.0f;
  Q.block<3, 3>(12, 12) = m_sigma_beta_f * dt;

  return Q;
}

void Mekf6::Update(const ControlInput &gyro_tilde,
                   const Observation &accel_tilde, double dt) {
  // subtract bias from gyro measurement
  ControlInput gyro = gyro_tilde - x_.segment<ControlInputDimension>(9);
  Observation accel = accel_tilde - x_.segment<ObservationDimension>(12);

  // propagate state to get q_minus
  q_hat_ =
      q_hat_ * Eigen::Quaterniond(1, x_(0) / 2.0f, x_(1) / 2.0f, x_(2) / 2.0f);
  q_hat_.normalize();

  // update state transition matrix
  Eigen::Matrix<double, StateDimension, StateDimension> F =
      Eigen::Matrix<double, StateDimension, StateDimension>::Zero();
  Eigen::Matrix<double, 3, 3> f_skew;
  f_skew << 0, -accel(2), accel(1), accel(2), 0, -accel(0), -accel(1), accel(0),
      0;

  F.block<3, 3>(0, 0) << 0, gyro(2), -gyro(1), -gyro(2), 0, gyro(0), gyro(1),
      -gyro(0), 0;
  F.block<3, 3>(0, 9) = -Eigen::Matrix<double, 3, 3>::Identity();
  F.block<3, 3>(3, 0) = -q_hat_.toRotationMatrix() * f_skew;
  F.block<3, 3>(3, 12) = -q_hat_.toRotationMatrix();
  F.block<3, 3>(6, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, StateDimension, StateDimension> Phi =
      Eigen::Matrix<double, StateDimension, StateDimension>::Identity() +
      F * dt;

  // predict the state and covariance
  P_ = Phi * P_ * Phi.transpose() + GetQMatrix(dt);
  x_ = Phi * x_;

  // update the kalman gain
  Eigen::Matrix<double, ObservationDimension, StateDimension> H =
      Eigen::Matrix<double, ObservationDimension, StateDimension>::Zero();
  Eigen::Vector<double, ObservationDimension> h =
      q_hat_.inverse().toRotationMatrix() *
      Eigen::Vector3d(0, 0, params_.gravity_constant);
  Eigen::Matrix<double, 3, 3> h_skew;
  h_skew << 0, -h(2), h(1), h(2), 0, -h(0), -h(1), h(0), 0;
  H.block<3, 3>(0, 0) = h_skew;

  Eigen::Matrix<double, StateDimension, ObservationDimension> K =
      P_ * H.transpose() * (H * P_ * H.transpose() + R_).inverse();

  // update the state and covariance
  x_ = x_ + K * H * x_;
  P_ = (Eigen::Matrix<double, StateDimension, StateDimension>::Identity() -
        K * H) *
       P_;
}
}  // namespace xmotion
