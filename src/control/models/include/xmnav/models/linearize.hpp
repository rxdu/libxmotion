/*
 * @file linearize.hpp
 * @brief Numeric linearization of Deriv-concept models.
 *
 * Central-difference Jacobians of the continuous dynamics about an
 * operating point — the standard bridge from the nonlinear models to
 * linear tools (SolveDlqr, StateFeedbackController): linearize, Euler-
 * discretize (A_d = I + A dt, B_d = B dt, consistent with the models'
 * Step), synthesize. Works for any model exposing
 * State Deriv(State, Control).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MODELS_LINEARIZE_HPP
#define XMNAV_MODELS_LINEARIZE_HPP

#include <eigen3/Eigen/Dense>

namespace xmotion {

template <typename Model>
struct Linearization {
  Eigen::Matrix<double, Model::kStateDim, Model::kStateDim> A;
  Eigen::Matrix<double, Model::kStateDim, Model::kControlDim> B;
};

template <typename Model>
Linearization<Model> LinearizeNumeric(const Model &model,
                                      const typename Model::State &x0,
                                      const typename Model::Control &u0,
                                      double eps = 1e-6) {
  Linearization<Model> lin;
  for (int j = 0; j < Model::kStateDim; ++j) {
    typename Model::State xp = x0, xm = x0;
    xp(j) += eps;
    xm(j) -= eps;
    lin.A.col(j) = (model.Deriv(xp, u0) - model.Deriv(xm, u0)) / (2.0 * eps);
  }
  for (int j = 0; j < Model::kControlDim; ++j) {
    typename Model::Control up = u0, um = u0;
    up(j) += eps;
    um(j) -= eps;
    lin.B.col(j) = (model.Deriv(x0, up) - model.Deriv(x0, um)) / (2.0 * eps);
  }
  return lin;
}

}  // namespace xmotion

#endif  // XMNAV_MODELS_LINEARIZE_HPP
