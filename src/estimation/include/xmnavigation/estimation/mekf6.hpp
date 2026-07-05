/*
 * @file mekf6.hpp
 * @date 3/31/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MEKF6_HPP
#define XMOTION_MEKF6_HPP

#include <eigen3/Eigen/Geometry>

namespace xmotion {
class Mekf6 {
 public:
  static constexpr int StateDimension = 15;
  static constexpr int ControlInputDimension = 3;
  static constexpr int ObservationDimension = 3;

  // state: [alpha, delta v, delta r, beta omega, beta f]
  using State = Eigen::Matrix<double, StateDimension, 1>;
  using ControlInput = Eigen::Matrix<double, ControlInputDimension, 1>;
  using Observation = Eigen::Matrix<double, ObservationDimension, 1>;

  using StateCovariance = Eigen::Matrix<double, StateDimension, StateDimension>;
  using ProcessNoiseCovariance =
      Eigen::Matrix<double, StateDimension, StateDimension>;
  using ObservationNoiseCovariance =
      Eigen::Matrix<double, ObservationDimension, ObservationDimension>;

  struct Params {
    Eigen::Quaterniond init_quaternion;

    State init_state;
    StateCovariance init_state_cov;
    ObservationNoiseCovariance init_observation_noise_cov;

    Eigen::Vector3d sigma_omega;
    Eigen::Vector3d sigma_f;
    Eigen::Vector3d sigma_beta_omega;
    Eigen::Vector3d sigma_beta_f;

    double gravity_constant = 9.81;
  };

 public:
  void Initialize(const Params &params);
  void Update(const ControlInput &gyro_tilde, const Observation &accel_tilde,
              double dt);

  const Eigen::Quaterniond &GetQuaternion() const { return q_hat_; }

 private:
  Eigen::Matrix<double, StateDimension, StateDimension> GetQMatrix(double dt);

  State x_;
  StateCovariance P_;
  ObservationNoiseCovariance R_;

  Params params_;
  Eigen::Quaterniond q_hat_;
};
}  // namespace xmotion

#endif  // XMOTION_MEKF6_HPP
