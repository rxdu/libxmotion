/*
 * @file balance_controller.cpp
 * @date 7/24/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/controller/balance_controller.hpp"

#include "quadprog++/QuadProg++.hh"
#include "math_utils/matrix.hpp"

namespace xmotion {
namespace {
double SolveQp(const Eigen::Matrix<double, 12, 12> _G,
               const Eigen::Matrix<double, 12, 1> _g0,
               const Eigen::MatrixXd& _C_eq, const Eigen::VectorXd& _c_eq0,
               const Eigen::MatrixXd& _C_ineq, const Eigen::VectorXd& _c_ineq0,
               Eigen::Matrix<double, 12, 1>& f) {
  quadprogpp::Matrix<double> G, C_eq, C_ineq;
  quadprogpp::Vector<double> g0, c_eq0, c_ineq0, x;

  int n = f.size();
  int m = _c_eq0.size();
  int p = _c_ineq0.size();

  G.resize(n, n);
  C_eq.resize(n, m);
  C_ineq.resize(n, p);
  g0.resize(n);
  c_eq0.resize(m);
  c_ineq0.resize(p);
  x.resize(n);

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      G[i][j] = _G(i, j);
    }
  }

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      C_eq[i][j] = (_C_eq.transpose())(i, j);
    }
  }

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < p; ++j) {
      C_ineq[i][j] = (_C_ineq.transpose())(i, j);
    }
  }

  for (int i = 0; i < n; ++i) {
    g0[i] = _g0[i];
  }

  for (int i = 0; i < m; ++i) {
    c_eq0[i] = _c_eq0[i];
  }

  for (int i = 0; i < p; ++i) {
    c_ineq0[i] = _c_ineq0[i];
  }

  double value =
      quadprogpp::solve_quadprog(G, g0, C_eq, c_eq0, C_ineq, c_ineq0, x);

  for (int i = 0; i < n; ++i) {
    f[i] = x[i];
  }

  return value;
}

Eigen::Matrix<double, 3, 4> Vec12ToVec34(Eigen::Matrix<double, 12, 1> vec12) {
  Eigen::Matrix<double, 3, 4> vec34;
  for (int i(0); i < 4; ++i) {
    vec34.col(i) = vec12.segment(3 * i, 3);
  }
  return vec34;
}
}  // namespace

BalanceController::BalanceController(
    const ControllerParams::BalanceControllerParams& params,
    std::shared_ptr<QuadrupedModel> robot_model)
    : params_(params), robot_model_(robot_model) {
  f_prev_ = Eigen::Matrix<double, 12, 1>::Zero();
}

Eigen::Matrix<double, 3, 4> BalanceController::ComputeFootForce(
    double mu, const Eigen::Matrix<double, 3, 1>& p_ddot,
    const Eigen::Matrix<double, 3, 1>& omega_dot, const Quaterniond& quat,
    const Eigen::Matrix<double, 3, 4>& foot_pos,
    const Eigen::Vector4d& foot_contact) {
  Eigen::Matrix<double, 3, 4> f_d;

  // construct A
  Eigen::Matrix<double, 6, 12> A = Eigen::Matrix<double, 6, 12>::Zero();
  for (int i = 0; i < 4; ++i) {
    A.block<3, 3>(0, i * 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Position3d p_gi = quat.toRotationMatrix() *
                      (foot_pos.col(i) - robot_model_->GetCogOffset());
    A.block<3, 3>(3, i * 3) = MathUtils::SkewSymmetric(p_gi);
  }

  // construct B
  Eigen::Matrix<double, 6, 1> b;
  b.block<3, 1>(0, 0) = robot_model_->GetMass() * (p_ddot - g_const);
  b.block<3, 1>(3, 0) = quat.toRotationMatrix() *
                        robot_model_->GetMomentOfInertia() *
                        quat.toRotationMatrix().transpose() * omega_dot;

  // formulate constraints
  int num_contact = 0;
  for (int i = 0; i < 4; ++i) {
    if (foot_contact(i) != 0) num_contact++;
  }
  Eigen::MatrixXd C_ineq = Eigen::MatrixXd::Zero(5 * num_contact, 12);
  Eigen::VectorXd c_ineq0 = Eigen::VectorXd::Zero(5 * num_contact, 1);
  Eigen::MatrixXd C_eq = Eigen::MatrixXd::Zero(3 * (4 - num_contact), 12);
  Eigen::VectorXd c_eq0 = Eigen::VectorXd::Zero(3 * (4 - num_contact), 1);
  Eigen::Matrix<double, 5, 3> fric_mat;
  fric_mat << 1, 0, mu, -1, 0, mu, 0, 1, mu, 0, -1, mu, 0, 0, 1;
  int eq_idx = 0, ineq_idx = 0;
  for (int i(0); i < 4; ++i) {
    if (foot_contact(i) != 0) {
      // with contact: add to inequality constraint
      C_ineq.block<5, 3>(5 * ineq_idx, 3 * i) = fric_mat;
      ineq_idx++;
    } else {
      C_eq.block<3, 3>(3 * eq_idx, 3 * i) =
          Eigen::Matrix<double, 3, 3>::Identity();
      eq_idx++;
    }
  }

  // solve QP
  Eigen::Matrix<double, 6, 6> S = params_.s.asDiagonal();
  Eigen::Matrix<double, 12, 12> W = params_.w.asDiagonal();
  Eigen::Matrix<double, 12, 12> U = params_.u.asDiagonal();

  Eigen::Matrix<double, 12, 12> G =
      A.transpose() * S * A + params_.alpha * W + params_.beta * U;
  Eigen::Matrix<double, 12, 1> g0 =
      -b.transpose() * S * A - params_.beta * f_prev_.transpose() * U;

  Eigen::Matrix<double, 12, 1> f = Eigen::Matrix<double, 12, 1>::Zero();
  double value = SolveQp(G, g0, C_eq, c_eq0, C_ineq, c_ineq0, f);

  // save for next iteration
  f_prev_ = f;

  return Vec12ToVec34(f);
}
}  // namespace xmotion