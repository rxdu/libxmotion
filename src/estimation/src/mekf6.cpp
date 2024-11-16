/*
 * mekf6.cpp
 *
 * Created on 3/31/24 8:16 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "estimation/mekf6.hpp"

#include <iostream>

namespace xmotion {
namespace {
Eigen::Matrix<double, 3, 3> SkewSymmetric(const Eigen::Vector3d &v) {
  Eigen::Matrix<double, 3, 3> m;
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}
}  // namespace

void Mekf6::Initialize(const Params &params) {
  params_ = params;
  q_hat_ = params.init_quaternion;
  x_ = params.init_state;
  P_ = params.init_state_cov;
  R_ = params.init_observation_noise_cov;
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

  // reset the error state
  x_ = Eigen::Matrix<double, StateDimension, 1>::Zero();

  std::cout << "gyro_tilde: " << gyro_tilde.transpose() << std::endl;
  std::cout << "accel_tilde: " << accel_tilde.transpose() << std::endl;
  std::cout << "gyro: " << gyro.transpose() << std::endl;
  std::cout << "accel: " << accel.transpose() << std::endl;

  // predict state by propagating gyro data as control input
  q_hat_ = Eigen::Quaterniond(
      q_hat_.coeffs() +
      0.5 * dt *
          (q_hat_ * Eigen::Quaterniond(0, gyro(0), gyro(1), gyro(2))).coeffs());
  q_hat_.normalize();

  // update state transition matrix
  Eigen::Matrix<double, StateDimension, StateDimension> F =
      Eigen::Matrix<double, StateDimension, StateDimension>::Zero();
  F.block<3, 3>(0, 0) = -SkewSymmetric(gyro);
  F.block<3, 3>(0, 9) = -Eigen::Matrix<double, 3, 3>::Identity();
  F.block<3, 3>(3, 0) = -q_hat_.toRotationMatrix() * SkewSymmetric(accel);
  F.block<3, 3>(3, 12) = -q_hat_.toRotationMatrix();
  F.block<3, 3>(6, 3) = Eigen::Matrix<double, 3, 3>::Identity();

  Eigen::Matrix<double, StateDimension, StateDimension> Phi =
      Eigen::Matrix<double, StateDimension, StateDimension>::Identity() +
      F * dt;

  //  std::cout << "x: \n" << x_.transpose() << std::endl;
  //  std::cout << "F: \n" << F << std::endl;
  //  std::cout << "Phi: \n" << Phi << std::endl;
  //  std::cout << "P^-: \n" << P_ << std::endl;
  //  std::cout << "Q_d: \n" << GetQMatrix(dt) << std::endl;

  // predict the state and covariance
  P_ = Phi * P_ * Phi.transpose() + GetQMatrix(dt);
  x_ = Phi * x_;

  //  std::cout << "P^-: \n" << P_ << std::endl;
  //  std::cout << "x^-: \n" << x_.block<3, 1>(0, 0).transpose() << std::endl;

  // update the kalman gain
  Eigen::Matrix<double, ObservationDimension, StateDimension> H =
      Eigen::Matrix<double, ObservationDimension, StateDimension>::Zero();
  Eigen::Matrix<double, ObservationDimension, 1> h =
      q_hat_.inverse().toRotationMatrix() *
      Eigen::Vector3d(0, 0, -params_.gravity_constant);
  H.block<3, 3>(0, 0) = SkewSymmetric(h);
  H.block<3, 3>(0, 12) = Eigen::Matrix<double, 3, 3>::Identity();

  Eigen::Matrix<double, StateDimension, ObservationDimension> K =
      P_ * H.transpose() * (H * P_ * H.transpose() + R_).inverse();

  //  std::cout << "K: \n" << K << std::endl;
  //  std::cout << "H: \n" << H << std::endl;

  // update the state and covariance
  Eigen::Vector3d delta_y =
      accel - q_hat_.inverse().toRotationMatrix() *
                  Eigen::Vector3d(0, 0, -params_.gravity_constant);

  //  std::cout << "delta y: \n" << delta_y.transpose() << std::endl;
  //  std::cout << "K * delta_y: \n" << (K * delta_y).transpose() << std::endl;

  x_ = x_ + K * delta_y;
  P_ = (Eigen::Matrix<double, StateDimension, StateDimension>::Identity() -
        K * H) *
       P_;

  //  std::cout << "x^+: \n" << x_.transpose() << std::endl;
  //  std::cout << "P^+: \n" << P_ << std::endl;

  // calculate q_hat_plus
  q_hat_ =
      q_hat_ * Eigen::Quaterniond(1, x_(0) / 2.0f, x_(1) / 2.0f, x_(2) / 2.0f);
  q_hat_.normalize();

  x_.block<3, 1>(0, 0) += x_.block<3, 1>(9, 0);
  x_.block<3, 1>(3, 0) += x_.block<3, 1>(12, 0);
}
}  // namespace xmotion
