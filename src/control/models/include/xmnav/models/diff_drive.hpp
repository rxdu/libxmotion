/*
 * @file diff_drive.hpp
 * @brief Kinematic differential-drive model for MPPI rollouts.
 *
 * State [x, y, theta] (m, m, rad), control [v, w] (m/s, rad/s), forward
 * Euler. This is the model class production wheeled MPPI deployments use
 * (Nav2); dynamics effects are absorbed by the box constraints.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_DIFF_DRIVE_HPP
#define XMNAV_MODELS_DIFF_DRIVE_HPP

#include <cmath>

#include <eigen3/Eigen/Dense>

#include "xmnav/models/model_core.hpp"

namespace xmotion {

// Wraps the shared raw-span core (model_core::DiffDriveStep) that the CUDA
// rollout backend also compiles — one implementation of the dynamics for
// both backends.
struct DiffDriveModel {
  static constexpr int kStateDim = 3;
  static constexpr int kControlDim = 2;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;

  State Step(const State &x, const Control &u, int /*t*/, double dt) const {
    State next;
    model_core::DiffDriveStep(x.data(), u.data(), dt, next.data());
    return next;
  }
};

}  // namespace xmotion

#endif  // XMNAV_MODELS_DIFF_DRIVE_HPP
