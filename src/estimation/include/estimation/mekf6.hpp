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
  static constexpr int ObservationDimension = 3;

  using State = Eigen::Vector<double, StateDimension>;
  using StateCovariance = Eigen::Matrix<double, StateDimension, StateDimension>;
  using ObservationCovariance =
      Eigen::Matrix<double, ObservationDimension, ObservationDimension>;

  struct Params {
    State init_state;
    StateCovariance init_state_cov;
    ObservationCovariance init_observation_cov;

    Eigen::Vector<double, ObservationDimension> gyro_bias;
    Eigen::Vector<double, ObservationDimension> accel_bias;

    Eigen::Vector<double, ObservationDimension> accel_obs_cov;
  };

 public:
  void Initialize(const Params &params);
  void Update(const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel,
              double dt);

 private:
  State state_;
  StateCovariance state_cov_;
  ObservationCovariance observation_cov_;
  Eigen::Vector<double, ObservationDimension> gyro_bias_;
  Eigen::Vector<double, ObservationDimension> accel_bias_;
};
}  // namespace xmotion

#endif  // XMOTION_MEKF6_HPP
