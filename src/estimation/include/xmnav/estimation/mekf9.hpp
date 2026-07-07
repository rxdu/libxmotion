/*
 * @file mekf9.hpp
 * @date 5/7/24
 * @brief Multiplicative extended Kalman filter, 9-DOF IMU
 *        (gyro + accel + magnetometer).
 *
 * 18-error-state MEKF estimating attitude plus gyro, accelerometer, and
 * magnetometer biases. The magnetometer observation makes heading (and the
 * gyro z-bias) observable, which the gravity-only Mekf6 cannot see. The
 * derivation is docs/typst/main.typ (18-state model with the magnetometer
 * measurement); conventions match Mekf6:
 *
 *  - q_hat is body-to-inertial (C^i_b = R(q_hat)).
 *  - Gravity is (0, 0, -g) in the inertial frame.
 *  - mag_reference is the local magnetic field in the inertial frame, in the
 *    same (arbitrary but consistent) units as the magnetometer output.
 *
 * Error state (18x1): [alpha, delta v, delta r, beta_omega, beta_f, beta_m],
 * folded into the nominal states and reset after every measurement update.
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MEKF_9_HPP_
#define XMOTION_MEKF_9_HPP_

#include <eigen3/Eigen/Geometry>

#include "xmbase/telemetry/telemetry.hpp"

namespace xmotion {
class Mekf9 {
 public:
  static constexpr int StateDimension = 18;
  static constexpr int ControlInputDimension = 3;
  static constexpr int ObservationDimension = 3;  // per vector observation

  // error state: [alpha, delta v, delta r, beta omega, beta f, beta m]
  using State = Eigen::Matrix<double, StateDimension, 1>;
  using ControlInput = Eigen::Matrix<double, ControlInputDimension, 1>;
  using Observation = Eigen::Matrix<double, ObservationDimension, 1>;

  using StateCovariance = Eigen::Matrix<double, StateDimension, StateDimension>;
  using ProcessNoiseCovariance =
      Eigen::Matrix<double, StateDimension, StateDimension>;
  using ObservationNoiseCovariance =
      Eigen::Matrix<double, ObservationDimension, ObservationDimension>;

  struct Params {
    Eigen::Quaterniond init_quaternion = Eigen::Quaterniond::Identity();
    Eigen::Vector3d init_gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_mag_bias = Eigen::Vector3d::Zero();

    StateCovariance init_state_cov = StateCovariance::Identity();
    ObservationNoiseCovariance accel_noise_cov =
        ObservationNoiseCovariance::Identity();
    ObservationNoiseCovariance mag_noise_cov =
        ObservationNoiseCovariance::Identity();

    // white-noise standard deviations
    Eigen::Vector3d sigma_omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigma_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigma_beta_omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigma_beta_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigma_beta_m = Eigen::Vector3d::Zero();

    double gravity_constant = 9.81;
    // local magnetic field in the inertial frame (magnitude sets the units
    // the magnetometer is expected to report)
    Eigen::Vector3d mag_reference = Eigen::Vector3d(1.0, 0.0, 0.0);

    // observation gates (<= 0 disables): accel by | ||a|| - g |, mag by
    // | ||m|| - ||mag_reference|| | (soft iron / interference rejection)
    double accel_gate_threshold = 0.5;
    double mag_gate_threshold = 0.2;
  };

 public:
  void Initialize(const Params &params);

  // One predict cycle plus gated accelerometer and magnetometer updates.
  // Returns false and leaves the state untouched on invalid input.
  bool Update(const ControlInput &gyro_tilde, const Observation &accel_tilde,
              const Observation &mag_tilde, double dt);

  const Eigen::Quaterniond &GetQuaternion() const { return q_hat_; }
  const Eigen::Vector3d &GetGyroBias() const { return b_omega_; }
  const Eigen::Vector3d &GetAccelBias() const { return b_f_; }
  const Eigen::Vector3d &GetMagBias() const { return b_m_; }
  const StateCovariance &GetCovariance() const { return P_; }

  bool LastUpdateUsedAccel() const { return last_accel_used_; }
  bool LastUpdateUsedMag() const { return last_mag_used_; }

 private:
  ProcessNoiseCovariance GetQMatrix(double dt) const;
  // one vector-observation update: residual dy against predicted h, with the
  // attitude block [h]x and the identity block at bias_index; folds + resets
  void ApplyVectorObservation(const Eigen::Vector3d &delta_y,
                              const Eigen::Vector3d &h, int bias_index,
                              const ObservationNoiseCovariance &R);

  StateCovariance P_ = StateCovariance::Identity();

  Params params_;
  // nominal states
  Eigen::Quaterniond q_hat_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d b_omega_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_f_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_m_ = Eigen::Vector3d::Zero();

  bool last_accel_used_ = false;
  bool last_mag_used_ = false;

  // pre-acquired wait-free telemetry handles (no-ops when unbound)
  telemetry::Gauge gyro_bias_gauge_ =
      telemetry::GetGauge("estimation.mekf9.gyro_bias_norm");
  telemetry::Counter accel_reject_counter_ =
      telemetry::GetCounter("estimation.mekf9.accel_gate_rejections");
  telemetry::Counter mag_reject_counter_ =
      telemetry::GetCounter("estimation.mekf9.mag_gate_rejections");
  telemetry::Counter invalid_input_counter_ =
      telemetry::GetCounter("estimation.mekf9.invalid_inputs");
};
}  // namespace xmotion

#endif  // XMOTION_MEKF_9_HPP_
