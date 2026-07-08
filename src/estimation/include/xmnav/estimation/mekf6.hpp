/*
 * @file mekf6.hpp
 * @date 3/31/24
 * @brief Multiplicative extended Kalman filter, 6-DOF IMU (gyro + accel).
 *
 * Error-state MEKF estimating attitude plus gyro/accelerometer biases from a
 * 6-axis IMU, using gravity as the vector observation. The derivation and
 * conventions live in docs/typst/main.typ (following Maley 2013 and Sola
 * 2017; the MEKF formulation originates with Lefferts, Markley & Shuster
 * 1982).
 *
 * Conventions:
 *  - q_hat is the body-to-inertial rotation (C^i_b = R(q_hat)).
 *  - Gravity is modeled as (0, 0, -g) in the inertial frame; a stationary,
 *    level accelerometer therefore reads approximately (0, 0, -g).
 *  - Gyro input in rad/s, accelerometer input in m/s^2.
 *
 * Error state (15x1): [alpha, delta v, delta r, beta_omega, beta_f]. The
 * error state is folded into the nominal states (quaternion + biases) after
 * every measurement update and then reset to zero.
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MEKF6_HPP
#define XMOTION_MEKF6_HPP

#include <eigen3/Eigen/Geometry>

#include "xmbase/telemetry/telemetry.hpp"

namespace xmotion {
class Mekf6 {
 public:
  static constexpr int StateDimension = 15;
  static constexpr int ControlInputDimension = 3;
  static constexpr int ObservationDimension = 3;

  // error state: [alpha, delta v, delta r, beta omega, beta f]
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

    StateCovariance init_state_cov = StateCovariance::Identity();
    ObservationNoiseCovariance init_observation_noise_cov =
        ObservationNoiseCovariance::Identity();

    // white-noise standard deviations (gyro, accel, and their bias walks)
    Eigen::Vector3d sigma_omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigma_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigma_beta_omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigma_beta_f = Eigen::Vector3d::Zero();

    double gravity_constant = 9.81;

    // reject the gravity observation when | ||accel|| - g | exceeds this
    // threshold (m/s^2): the vector observation is only valid when the body
    // is not accelerating. Set <= 0 to disable gating.
    double accel_gate_threshold = 0.5;
  };

 public:
  void Initialize(const Params &params);

  // One predict + (gated) gravity-update cycle. Returns false and leaves the
  // state untouched if the inputs are invalid (non-finite values, dt <= 0).
  bool Update(const ControlInput &gyro_tilde, const Observation &accel_tilde,
              double dt);

  const Eigen::Quaterniond &GetQuaternion() const { return q_hat_; }
  const Eigen::Vector3d &GetGyroBias() const { return b_omega_; }
  const Eigen::Vector3d &GetAccelBias() const { return b_f_; }
  const StateCovariance &GetCovariance() const { return P_; }

  // true if the last Update() applied the gravity observation (i.e. the
  // accelerometer norm passed the gate)
  bool LastUpdateUsedObservation() const { return last_obs_used_; }

 private:
  ProcessNoiseCovariance GetQMatrix(double dt) const;

  StateCovariance P_ = StateCovariance::Identity();
  ObservationNoiseCovariance R_ = ObservationNoiseCovariance::Identity();

  Params params_;
  // nominal states
  Eigen::Quaterniond q_hat_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d b_omega_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_f_ = Eigen::Vector3d::Zero();

  bool last_obs_used_ = false;

  // pre-acquired wait-free telemetry handles (no-ops when unbound)
  telemetry::Gauge innovation_gauge_ =
      telemetry::GetGauge("estimation.mekf6.innovation_norm");
  telemetry::Gauge gyro_bias_gauge_ =
      telemetry::GetGauge("estimation.mekf6.gyro_bias_norm");
  telemetry::Counter gate_reject_counter_ =
      telemetry::GetCounter("estimation.mekf6.gate_rejections");
  telemetry::Counter invalid_input_counter_ =
      telemetry::GetCounter("estimation.mekf6.invalid_inputs");
};
}  // namespace xmotion

#endif  // XMOTION_MEKF6_HPP
