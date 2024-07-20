/*
 * @file simple_estimator.hpp
 * @date 7/17/24
 * @brief a simple linear kalman filter from unitree book
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_SIMPLE_ESTIMATOR_HPP
#define QUADRUPED_MOTION_SIMPLE_ESTIMATOR_HPP

#include "interface/type/geometry_types.hpp"

#include "quadruped/system_config.hpp"
#include "quadruped/robot_model/quadruped_model.hpp"

namespace xmotion {
class SimpleEstimator {
  using State = Eigen::Matrix<double, 18, 1>;
  using StateCovariance = Eigen::Matrix<double, 18, 18>;
  using MeasurementCovariance = Eigen::Matrix<double, 28, 28>;
  using Mesaurement = Eigen::Matrix<double, 28, 1>;
  using Control = Eigen::Matrix<double, 3, 1>;

 public:
  struct Params {
    State x0;
    StateCovariance p0;
    MeasurementCovariance r0;
    double gravity_constant = 9.81;
  };

 public:
  SimpleEstimator(const EstimatorSettings& settings,
                  std::shared_ptr<QuadrupedModel> robot_model);
  ~SimpleEstimator() = default;

  // public methods
  void Initialize(const Params& params);
  void Update(const QuadrupedModel::SensorData& sensor_data, double dt);

 private:
  EstimatorSettings settings_;
  std::shared_ptr<QuadrupedModel> robot_model_;

  State x_hat_;
  Control u_;
  Mesaurement y_;
  Mesaurement y_hat_;
  Eigen::Matrix<double, 18, 18> A_;
  Eigen::Matrix<double, 18, 3> B_;
  Eigen::Matrix<double, 28, 18> C_;
  StateCovariance P_;
  StateCovariance Q_;
  MeasurementCovariance R_;

  Eigen::Matrix<double, 4, 1> foot_height;

  Params params_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_SIMPLE_ESTIMATOR_HPP