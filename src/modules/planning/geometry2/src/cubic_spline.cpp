/*
 * cublic_spline.cpp
 *
 * Created on: Oct 13, 2018 11:08
 * Description:
 *
 * Reference:
 * [1] Introduction to Numerical Analysis, Josef Stoer, R. Bulirsch, Springer
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "geometry/cubic_spline.hpp"

#include <memory>
#include <iostream>
#include <algorithm>
#include <cassert>

namespace robotnav {
CubicSpline::CubicSpline(const std::vector<Knot> &knots) : knots_(knots) {
  Interpolate();
}

CubicSpline::CubicSpline(double fp0, double fpn, const std::vector<Knot> &knots)
    : knots_(knots) {
  Interpolate(fp0, fpn);
}

void CubicSpline::Interpolate(const std::vector<Knot> &knots) {
  // replace existing knots if a non-empty set of Knots is passed in
  if (!knots.empty()) knots_ = knots;

  /* interpolation */
  const size_t n = knots_.size() - 1;

  // no interpolation if size < 3
  if (n < 2) return;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n + 1);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(n + 1);
  Eigen::VectorXd h = Eigen::VectorXd::Zero(n);

  for (size_t i = 0; i < knots_.size(); ++i) {
    x(i) = knots_[i].x();
    a(i) = knots_[i].y();
  }

  for (size_t i = 0; i < n; ++i) {
    h(i) = x(i + 1) - x(i);
  }

  Eigen::VectorXd alpha = Eigen::VectorXd::Zero(n);

  for (size_t i = 1; i < n; ++i) {
    alpha(i) = 3 / h(i) * (a(i + 1) - a(i)) - 3 / h(i - 1) * (a(i) - a(i - 1));
  }

  Eigen::VectorXd miu = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(n + 1);
  Eigen::VectorXd z = Eigen::VectorXd::Zero(n + 1);
  l(0) = 1;
  miu(0) = 0;
  z(0) = 0;
  for (size_t i = 1; i < n; ++i) {
    l(i) = 2 * (x(i + 1) - x(i - 1)) - h(i - 1) * miu(i - 1);
    miu(i) = h(i) / l(i);
    z(i) = (alpha(i) - h(i - 1) * z(i - 1)) / l(i);
  }

  Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n + 1);
  Eigen::VectorXd d = Eigen::VectorXd::Zero(n);
  l(n) = 1;
  z(n) = 0;
  c(n) = 0;
  for (int64_t j = n - 1; j >= 0; --j) {
    c(j) = z(j) - miu(j) * c(j + 1);
    b(j) = (a(j + 1) - a(j)) / h(j) - h(j) * (c(j + 1) + 2 * c(j)) / 3;
    d(j) = (c(j + 1) - c(j)) / (3 * h(j));
  }
  std::cout << "result: " << std::endl;
  std::cout << "a: " << a.transpose() << std::endl;
  std::cout << "b: " << b.transpose() << std::endl;
  std::cout << "c: " << c.transpose() << std::endl;
  std::cout << "d: " << d.transpose() << std::endl;
}

void CubicSpline::Interpolate(double fp0, double fpn,
                              const std::vector<Knot> &knots) {
  // replace existing knots if a non-empty set of Knots is passed in
  if (!knots.empty()) knots_ = knots;

  /* interpolation */
  const size_t n = knots_.size() - 1;

  // no interpolation if size < 3
  if (n < 2) return;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n + 1);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(n + 1);
  Eigen::VectorXd h = Eigen::VectorXd::Zero(n);

  for (size_t i = 0; i < knots_.size(); ++i) {
    x(i) = knots_[i].x();
    a(i) = knots_[i].y();
  }

  for (size_t i = 0; i < n; ++i) {
    h(i) = x(i + 1) - x(i);
  }

  Eigen::VectorXd alpha = Eigen::VectorXd::Zero(n + 1);
  alpha(0) = 3 * (a(1) - a(0)) / h(0) - 3 * fp0;
  alpha(n) = 3 * fpn - 3 * (a(n) - a(n - 1)) / h(n - 1);

  for (size_t i = 1; i < n; ++i) {
    alpha(i) = 3 / h(i) * (a(i + 1) - a(i)) - 3 / h(i - 1) * (a(i) - a(i - 1));
  }

  Eigen::VectorXd miu = Eigen::VectorXd::Zero(n + 1);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(n + 1);
  Eigen::VectorXd z = Eigen::VectorXd::Zero(n + 1);

  Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n + 1);
  Eigen::VectorXd d = Eigen::VectorXd::Zero(n);

  l(0) = 2 * h(0);
  miu(0) = 0.5;
  z(0) = alpha(0) / l(0);

  for (size_t i = 1; i < n; ++i) {
    l(i) = 2 * (x(i + 1) - x(i - 1)) - h(i - 1) * miu(i - 1);
    miu(i) = h(i) / l(i);
    z(i) = (alpha(i) - h(i - 1) * z(i - 1)) / l(i);
  }

  l(n) = h(n - 1) * (2 - miu(n - 1));
  z(n) = (alpha(n) - h(n - 1) * z(n - 1)) / l(n);
  c(n) = z(n);

  for (int64_t j = n - 1; j >= 0; --j) {
    c(j) = z(j) - miu(j) * c(j + 1);
    b(j) = (a(j + 1) - a(j)) / h(j) - h(j) * (c(j + 1) + 2 * c(j)) / 3;
    d(j) = (c(j + 1) - c(j)) / (3 * h(j));
  }

  std::cout << "result: " << std::endl;
  std::cout << "a: " << a.transpose() << std::endl;
  std::cout << "b: " << b.transpose() << std::endl;
  std::cout << "c: " << c.transpose() << std::endl;
  std::cout << "d: " << d.transpose() << std::endl;
}

double CubicSpline::Evaluate(double x, uint32_t derivative) const {
  assert(derivative <= 2);

  //   switch (derivative) {
  //     case 0:
  //       return Evaluate(x);
  //     case 1:
  //       return gsl_spline_eval_deriv(spline_, x, accel_);
  //     case 2:
  //       return gsl_spline_eval_deriv2(spline_, x, accel_);
  //     default:
  //       return 0;
  //   }
  return 0.0;
}
}  // namespace robotnav
