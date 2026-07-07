/*
 * mekf6.cpp
 *
 * Created on 3/31/24 8:16 PM
 * Description: error-state MEKF update cycle. Equation references point to
 * docs/typst/main.typ (which follows Maley 2013 / Sola 2017).
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "xmnav/estimation/mekf6.hpp"

#include <cmath>

#include "xmbase/math/matrix_utils.hpp"

namespace xmotion {
namespace {
using MathUtils::SkewSymmetric;

using Mat3 = Eigen::Matrix<double, 3, 3>;

Mat3 DiagSq(const Eigen::Vector3d &sigma) {
  return sigma.cwiseProduct(sigma).asDiagonal();
}
}  // namespace

void Mekf6::Initialize(const Params &params) {
  params_ = params;
  q_hat_ = params.init_quaternion.normalized();
  b_omega_ = params.init_gyro_bias;
  b_f_ = params.init_accel_bias;
  P_ = params.init_state_cov;
  R_ = params.init_observation_noise_cov;
  last_obs_used_ = false;
}

// Discrete process noise Q_d = int_0^dt Phi(tau) Q_c Phi(tau)^T dtau for the
// error-state dynamics, first-order Phi. Symmetric by construction: the
// lower-triangular blocks mirror the upper ones.
Mekf6::ProcessNoiseCovariance Mekf6::GetQMatrix(double dt) const {
  ProcessNoiseCovariance Q = ProcessNoiseCovariance::Zero();

  const Mat3 s_w = DiagSq(params_.sigma_omega);
  const Mat3 s_f = DiagSq(params_.sigma_f);
  const Mat3 s_bw = DiagSq(params_.sigma_beta_omega);
  const Mat3 s_bf = DiagSq(params_.sigma_beta_f);

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double dt5 = dt4 * dt;

  // attitude error <-> gyro bias
  Q.block<3, 3>(0, 0) = s_w * dt + s_bw * dt3 / 3.0;
  Q.block<3, 3>(0, 9) = -s_bw * dt2 / 2.0;
  Q.block<3, 3>(9, 0) = Q.block<3, 3>(0, 9).transpose();
  Q.block<3, 3>(9, 9) = s_bw * dt;

  // velocity/position error <-> accel bias
  Q.block<3, 3>(3, 3) = s_f * dt + s_bf * dt3 / 3.0;
  Q.block<3, 3>(3, 6) = s_f * dt2 / 2.0 + s_bf * dt4 / 8.0;
  Q.block<3, 3>(6, 3) = Q.block<3, 3>(3, 6).transpose();
  Q.block<3, 3>(3, 12) = -s_bf * dt2 / 2.0;
  Q.block<3, 3>(12, 3) = Q.block<3, 3>(3, 12).transpose();
  Q.block<3, 3>(6, 6) = s_f * dt3 / 3.0 + s_bf * dt5 / 20.0;
  Q.block<3, 3>(6, 12) = -s_bf * dt3 / 6.0;
  Q.block<3, 3>(12, 6) = Q.block<3, 3>(6, 12).transpose();
  Q.block<3, 3>(12, 12) = s_bf * dt;

  return Q;
}

bool Mekf6::Update(const ControlInput &gyro_tilde,
                   const Observation &accel_tilde, double dt) {
  if (!(dt > 0.0) || !gyro_tilde.allFinite() || !accel_tilde.allFinite()) {
    return false;
  }

  // bias-corrected measurements against the nominal bias states
  const Eigen::Vector3d gyro = gyro_tilde - b_omega_;
  const Eigen::Vector3d accel = accel_tilde - b_f_;

  // --- propagate the nominal attitude with the gyro (first-order) ---
  q_hat_ = Eigen::Quaterniond(
      q_hat_.coeffs() +
      0.5 * dt *
          (q_hat_ * Eigen::Quaterniond(0, gyro(0), gyro(1), gyro(2))).coeffs());
  q_hat_.normalize();

  // --- propagate the error-state covariance ---
  // (the error-state mean is zero after every fold/reset, so only P moves)
  const Mat3 C_i_b = q_hat_.toRotationMatrix();  // body -> inertial

  StateCovariance F = StateCovariance::Zero();
  F.block<3, 3>(0, 0) = -SkewSymmetric(gyro);
  F.block<3, 3>(0, 9) = -Mat3::Identity();
  F.block<3, 3>(3, 0) = -C_i_b * SkewSymmetric(accel);
  F.block<3, 3>(3, 12) = -C_i_b;
  F.block<3, 3>(6, 3) = Mat3::Identity();

  const StateCovariance Phi = StateCovariance::Identity() + F * dt;
  P_ = Phi * P_ * Phi.transpose() + GetQMatrix(dt);

  // --- gravity observation (gated: only valid when not accelerating) ---
  const Eigen::Vector3d g_i(0.0, 0.0, -params_.gravity_constant);
  last_obs_used_ =
      params_.accel_gate_threshold <= 0.0 ||
      std::abs(accel.norm() - params_.gravity_constant) <=
          params_.accel_gate_threshold;
  if (!last_obs_used_) {
    P_ = 0.5 * (P_ + P_.transpose());
    return true;  // prediction-only cycle
  }

  const Eigen::Vector3d h = q_hat_.conjugate() * g_i;  // predicted accel

  Eigen::Matrix<double, ObservationDimension, StateDimension> H =
      Eigen::Matrix<double, ObservationDimension, StateDimension>::Zero();
  H.block<3, 3>(0, 0) = SkewSymmetric(h);
  H.block<3, 3>(0, 12) = Mat3::Identity();

  const Eigen::Matrix<double, ObservationDimension, ObservationDimension> S =
      H * P_ * H.transpose() + R_;
  const Eigen::Matrix<double, StateDimension, ObservationDimension> K =
      (S.ldlt().solve(H * P_)).transpose();

  // innovation and error-state estimate (prior error mean is zero)
  const Eigen::Vector3d delta_y = accel - h;
  const State x = K * delta_y;

  // Joseph-form covariance update, then re-symmetrize
  const StateCovariance IKH = StateCovariance::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());

  // --- fold the error estimate into the nominal states, reset the error ---
  q_hat_ = q_hat_ * Eigen::Quaterniond(1.0, x(0) / 2.0, x(1) / 2.0, x(2) / 2.0);
  q_hat_.normalize();
  b_omega_ += x.segment<3>(9);
  b_f_ += x.segment<3>(12);
  // (delta v / delta r fold when nominal velocity/position tracking lands)

  return true;
}
}  // namespace xmotion
