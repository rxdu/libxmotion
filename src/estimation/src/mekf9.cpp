/*
 * mekf9.cpp
 *
 * Created on 5/7/24 11:08 PM
 * Description: 18-state error-state MEKF cycle (gyro + accel + mag), per the
 * derivation in docs/typst/main.typ. Vector observations are processed
 * sequentially with an error-state fold + reset after each, so the second
 * observation linearizes about the freshest attitude.
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "xmnav/estimation/mekf9.hpp"

#include <cmath>

#include "xmnav/estimation/matrix_utils.hpp"

namespace xmotion {
namespace {
using MathUtils::SkewSymmetric;

using Mat3 = Eigen::Matrix<double, 3, 3>;

Mat3 DiagSq(const Eigen::Vector3d &sigma) {
  return sigma.cwiseProduct(sigma).asDiagonal();
}
}  // namespace

void Mekf9::Initialize(const Params &params) {
  params_ = params;
  q_hat_ = params.init_quaternion.normalized();
  b_omega_ = params.init_gyro_bias;
  b_f_ = params.init_accel_bias;
  b_m_ = params.init_mag_bias;
  P_ = params.init_state_cov;
  last_accel_used_ = false;
  last_mag_used_ = false;
}

// Discrete process noise per the corrected derivation; the magnetometer bias
// walk only touches its own diagonal block.
Mekf9::ProcessNoiseCovariance Mekf9::GetQMatrix(double dt) const {
  ProcessNoiseCovariance Q = ProcessNoiseCovariance::Zero();

  const Mat3 s_w = DiagSq(params_.sigma_omega);
  const Mat3 s_f = DiagSq(params_.sigma_f);
  const Mat3 s_bw = DiagSq(params_.sigma_beta_omega);
  const Mat3 s_bf = DiagSq(params_.sigma_beta_f);
  const Mat3 s_bm = DiagSq(params_.sigma_beta_m);

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double dt5 = dt4 * dt;

  Q.block<3, 3>(0, 0) = s_w * dt + s_bw * dt3 / 3.0;
  Q.block<3, 3>(0, 9) = -s_bw * dt2 / 2.0;
  Q.block<3, 3>(9, 0) = Q.block<3, 3>(0, 9).transpose();
  Q.block<3, 3>(9, 9) = s_bw * dt;

  Q.block<3, 3>(3, 3) = s_f * dt + s_bf * dt3 / 3.0;
  Q.block<3, 3>(3, 6) = s_f * dt2 / 2.0 + s_bf * dt4 / 8.0;
  Q.block<3, 3>(6, 3) = Q.block<3, 3>(3, 6).transpose();
  Q.block<3, 3>(3, 12) = -s_bf * dt2 / 2.0;
  Q.block<3, 3>(12, 3) = Q.block<3, 3>(3, 12).transpose();
  Q.block<3, 3>(6, 6) = s_f * dt3 / 3.0 + s_bf * dt5 / 20.0;
  Q.block<3, 3>(6, 12) = -s_bf * dt3 / 6.0;
  Q.block<3, 3>(12, 6) = Q.block<3, 3>(6, 12).transpose();
  Q.block<3, 3>(12, 12) = s_bf * dt;

  Q.block<3, 3>(15, 15) = s_bm * dt;

  return Q;
}

void Mekf9::ApplyVectorObservation(const Eigen::Vector3d &delta_y,
                                   const Eigen::Vector3d &h, int bias_index,
                                   const ObservationNoiseCovariance &R) {
  Eigen::Matrix<double, ObservationDimension, StateDimension> H =
      Eigen::Matrix<double, ObservationDimension, StateDimension>::Zero();
  H.block<3, 3>(0, 0) = SkewSymmetric(h);
  H.block<3, 3>(0, bias_index) = Mat3::Identity();

  const Eigen::Matrix<double, ObservationDimension, ObservationDimension> S =
      H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, StateDimension, ObservationDimension> K =
      (S.ldlt().solve(H * P_)).transpose();

  const State x = K * delta_y;  // prior error mean is zero after every reset

  const StateCovariance IKH = StateCovariance::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());

  // fold into the nominal states, reset the error state
  q_hat_ = q_hat_ * Eigen::Quaterniond(1.0, x(0) / 2.0, x(1) / 2.0, x(2) / 2.0);
  q_hat_.normalize();
  b_omega_ += x.segment<3>(9);
  b_f_ += x.segment<3>(12);
  b_m_ += x.segment<3>(15);
}

bool Mekf9::Update(const ControlInput &gyro_tilde,
                   const Observation &accel_tilde, const Observation &mag_tilde,
                   double dt) {
  XM_SPAN("estimation.mekf9.update");
  if (!(dt > 0.0) || !gyro_tilde.allFinite() || !accel_tilde.allFinite() ||
      !mag_tilde.allFinite()) {
    invalid_input_counter_.Add();
    return false;
  }

  const Eigen::Vector3d gyro = gyro_tilde - b_omega_;
  const Eigen::Vector3d accel = accel_tilde - b_f_;
  const Eigen::Vector3d mag = mag_tilde - b_m_;

  // --- propagate the nominal attitude ---
  q_hat_ = Eigen::Quaterniond(
      q_hat_.coeffs() +
      0.5 * dt *
          (q_hat_ * Eigen::Quaterniond(0, gyro(0), gyro(1), gyro(2))).coeffs());
  q_hat_.normalize();

  // --- propagate the error-state covariance ---
  const Mat3 C_i_b = q_hat_.toRotationMatrix();

  StateCovariance F = StateCovariance::Zero();
  F.block<3, 3>(0, 0) = -SkewSymmetric(gyro);
  F.block<3, 3>(0, 9) = -Mat3::Identity();
  F.block<3, 3>(3, 0) = -C_i_b * SkewSymmetric(accel);
  F.block<3, 3>(3, 12) = -C_i_b;
  F.block<3, 3>(6, 3) = Mat3::Identity();

  const StateCovariance Phi = StateCovariance::Identity() + F * dt;
  P_ = Phi * P_ * Phi.transpose() + GetQMatrix(dt);
  P_ = 0.5 * (P_ + P_.transpose());

  // --- gravity observation (gated) ---
  // Gate on the RAW specific-force magnitude, not the bias-corrected
  // |accel - b_f|. The gate is a quasi-static detector ("is |a| ~ g right
  // now?"), which is a property of the measurement, not of the estimated bias.
  // Keying it on b_f creates a feedback lock: if b_f diverges (e.g. it absorbed
  // error while the accel was gated out during motion), the bias-corrected
  // magnitude no longer looks like gravity, so the filter rejects the
  // accelerometer even while stationary with clean gravity -- and since the
  // accelerometer is the only observation that corrects both attitude and b_f,
  // it can never recover. The update below still uses the bias-corrected accel.
  const Eigen::Vector3d g_i(0.0, 0.0, -params_.gravity_constant);
  last_accel_used_ =
      params_.accel_gate_threshold <= 0.0 ||
      std::abs(accel_tilde.norm() - params_.gravity_constant) <=
          params_.accel_gate_threshold;
  if (last_accel_used_) {
    const Eigen::Vector3d h_a = q_hat_.conjugate() * g_i;
    ApplyVectorObservation(accel - h_a, h_a, 12, params_.accel_noise_cov);
  } else {
    accel_reject_counter_.Add();
  }

  // --- magnetometer observation (gated) ---
  // Same rationale as the accelerometer: gate on the raw magnitude so a diverged
  // magnetometer bias b_m cannot lock the magnetometer out.
  last_mag_used_ = params_.mag_gate_threshold <= 0.0 ||
                   std::abs(mag_tilde.norm() - params_.mag_reference.norm()) <=
                       params_.mag_gate_threshold;
  if (last_mag_used_) {
    const Eigen::Vector3d h_m = q_hat_.conjugate() * params_.mag_reference;
    ApplyVectorObservation(mag - h_m, h_m, 15, params_.mag_noise_cov);
  } else {
    mag_reject_counter_.Add();
  }
  gyro_bias_gauge_.Set(b_omega_.norm());

  return true;
}
}  // namespace xmotion
