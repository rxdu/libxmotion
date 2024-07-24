/*
 * @file sim_dummy_estimator.hpp
 * @date 7/23/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_SIM_DUMMY_ESTIMATOR_HPP
#define QUADRUPED_MOTION_SIM_DUMMY_ESTIMATOR_HPP

#include "quadruped/estimator/estimator_interface.hpp"

namespace xmotion {
class SimDummyEstimator final : public EstimatorInterface {
 public:
  SimDummyEstimator() = default;
  ~SimDummyEstimator() = default;

  void Update(const QuadrupedModel::SensorData& sensor_data,
              double dt) override;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_SIM_DUMMY_ESTIMATOR_HPP