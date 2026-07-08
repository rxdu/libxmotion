/*
 * @file dlqr.hpp
 * @brief Discrete-time LQR gain synthesis (Riccati fixed-point iteration).
 *
 * Solves the infinite-horizon discrete-time LQR problem for
 * x_{k+1} = A x_k + B u_k, cost sum(x'Qx + u'Ru), by iterating the
 * Riccati recursion to its fixed point:
 *
 *   K = (R + B'PB)^{-1} B'PA
 *   P <- Q + K'RK + (A - BK)' P (A - BK)     (Joseph-stable form)
 *
 * ~30 dependency-free lines so state-feedback gains (state_feedback.hpp)
 * can be designed in-repo. Requires the usual LQR assumptions
 * ((A,B) stabilizable, Q >= 0, R > 0); on non-convergence within
 * max_iterations the result carries converged = false — check it.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_PID_DLQR_HPP
#define XMNAV_PID_DLQR_HPP

#include <eigen3/Eigen/Dense>

namespace xmotion {

template <int StateDim, int ControlDim>
struct DlqrResult {
  Eigen::Matrix<double, ControlDim, StateDim> K;
  Eigen::Matrix<double, StateDim, StateDim> P;
  bool converged = false;
  int iterations = 0;
};

template <int StateDim, int ControlDim>
DlqrResult<StateDim, ControlDim> SolveDlqr(
    const Eigen::Matrix<double, StateDim, StateDim> &A,
    const Eigen::Matrix<double, StateDim, ControlDim> &B,
    const Eigen::Matrix<double, StateDim, StateDim> &Q,
    const Eigen::Matrix<double, ControlDim, ControlDim> &R,
    double tolerance = 1e-10, int max_iterations = 10000) {
  DlqrResult<StateDim, ControlDim> result;
  result.P = Q;
  for (int i = 0; i < max_iterations; ++i) {
    const Eigen::Matrix<double, ControlDim, ControlDim> S =
        R + B.transpose() * result.P * B;
    result.K = S.ldlt().solve(B.transpose() * result.P * A);
    const Eigen::Matrix<double, StateDim, StateDim> Acl =
        A - B * result.K;
    const Eigen::Matrix<double, StateDim, StateDim> P_next =
        Q + result.K.transpose() * R * result.K +
        Acl.transpose() * result.P * Acl;
    const double delta = (P_next - result.P).cwiseAbs().maxCoeff();
    result.P = P_next;
    result.iterations = i + 1;
    if (delta < tolerance) {
      result.converged = true;
      break;
    }
  }
  return result;
}

}  // namespace xmotion

#endif  // XMNAV_PID_DLQR_HPP
