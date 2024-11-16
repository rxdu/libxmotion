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

namespace xmotion {
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

  coefficients_ = Eigen::MatrixXd::Zero(n, 4);

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

  // copy over results
  coefficients_.block(0, 0, n, 1) = a.block(0, 0, n, 1);
  coefficients_.block(0, 1, n, 1) = b;
  coefficients_.block(0, 2, n, 1) = c.block(0, 0, n, 1);
  coefficients_.block(0, 3, n, 1) = d;

  //   std::cout << "result: " << std::endl;
  //   std::cout << "a: " << a.transpose() << std::endl;
  //   std::cout << "b: " << b.transpose() << std::endl;
  //   std::cout << "c: " << c.transpose() << std::endl;
  //   std::cout << "d: " << d.transpose() << std::endl;
  //   std::cout << "------" << std::endl;
  //   std::cout << "coefficients: \n" << coefficients_ << std::endl;
}

void CubicSpline::Interpolate(double fp0, double fpn,
                              const std::vector<Knot> &knots) {
  // replace existing knots if a non-empty set of Knots is passed in
  if (!knots.empty()) knots_ = knots;

  /* interpolation */
  const size_t n = knots_.size() - 1;

  // no interpolation if size < 3
  if (n < 2) return;

  coefficients_ = Eigen::MatrixXd::Zero(n, 4);

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

  // copy over results
  coefficients_.block(0, 0, n, 1) = a.block(0, 0, n, 1);
  coefficients_.block(0, 1, n, 1) = b;
  coefficients_.block(0, 2, n, 1) = c.block(0, 0, n, 1);
  coefficients_.block(0, 3, n, 1) = d;

  //   std::cout << "result: " << std::endl;
  //   std::cout << "a: " << a.transpose() << std::endl;
  //   std::cout << "b: " << b.transpose() << std::endl;
  //   std::cout << "c: " << c.transpose() << std::endl;
  //   std::cout << "d: " << d.transpose() << std::endl;
  //   std::cout << "------" << std::endl;
  //   std::cout << "coefficients: \n" << coefficients_ << std::endl;
}

double CubicSpline::Evaluate(double x, uint32_t derivative) const {
  assert(derivative <= 3 && x >= knots_.front().x() && x <= knots_.back().x());

  if (derivative > 3 || x < knots_.front().x() || x > knots_.back().x())
    return std::numeric_limits<double>::signaling_NaN();

  // find segment
  uint32_t idx = 0;
  for (int i = 0; i < knots_.size(); ++i) {
    if (x >= knots_[i].x() && x <= knots_[i + 1].x()) {
      idx = i;
      break;
    }
  }

  // then evaluate
  double error = x - knots_[idx].x();
  if (derivative == 0) {
    return coefficients_(idx, 0) + coefficients_(idx, 1) * error +
           coefficients_(idx, 2) * error * error +
           coefficients_(idx, 3) * error * error * error;
  } else if (derivative == 1) {
    return coefficients_(idx, 1) + 2 * coefficients_(idx, 2) * error +
           3 * coefficients_(idx, 3) * error * error;
  } else if (derivative == 2) {
    return 2 * coefficients_(idx, 2) + 6 * coefficients_(idx, 3) * error;
  } else if (derivative == 3) {
    return 6 * coefficients_(idx, 3);
  } else {
    return std::numeric_limits<double>::signaling_NaN();
  }
}

//---------------------------------------------------------------------------//

#ifdef ENABLE_VISUAL
void DrawCubicSpline(quickviz::CvCanvas &canvas, const CubicSpline &spline, double step,
                     cv::Scalar ln_color, int32_t thickness) {
  std::vector<cv::Point2d> pts;
  std::vector<CubicSpline::Knot> knots(spline.GetKnots());
  for (double x = knots.front().x(); x < knots.back().x(); x += step)
    pts.emplace_back(x, spline.Evaluate(x));

  std::cout << "intermediate points: " << pts.size() << std::endl;

  for (std::size_t i = 0; i < pts.size() - 1; ++i)
    canvas.DrawLine({pts[i].x, pts[i].y}, {pts[i + 1].x, pts[i + 1].y},
                    ln_color, thickness);
}
#endif
}  // namespace xmotion
