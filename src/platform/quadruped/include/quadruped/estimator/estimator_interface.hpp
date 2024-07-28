/*
 * @file estimator_interface.hpp
 * @date 7/23/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_ESTIMATOR_INTERFACE_HPP
#define QUADRUPED_MOTION_ESTIMATOR_INTERFACE_HPP

#include "quadruped/robot_model/quadruped_model.hpp"

namespace xmotion {
class EstimatorInterface {
 public:
  virtual ~EstimatorInterface() = default;

  virtual void Update(const QuadrupedModel::SensorData& sensor_data,
                      double dt) = 0;

  virtual QuadrupedModel::AllJointVar GetEstimatedJointPosition() const = 0;
  virtual QuadrupedModel::AllJointVar GetEstimatedJointVelocity() const = 0;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_ESTIMATOR_INTERFACE_HPP
