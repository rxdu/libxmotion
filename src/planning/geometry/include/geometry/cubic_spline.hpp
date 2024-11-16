/*
 * cubic_spline.hpp
 *
 * Created on: Oct 13, 2018 11:02
 * Description: an implementation of Cubic Spline
 *  S_j(x) = a_j + b_j(x - x_j) + c_j(x - x_j)^2 + d_j(x - x_j)^3
 *  - natual boundary: S''(x_0) = S''(x_n) = 0
 *  - clamped boundary: S'(x_0) = f'(x_0), S'(x_n) = f'(x_n)
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include <vector>

#include "geometry/simple_point.hpp"

#ifdef ENABLE_VISUAL
#include "cvdraw/cvdraw.hpp"
#endif

namespace xmotion {
class CubicSpline {
 public:
  using Knot = SimplePoint2;
  enum class Type { Natual = 0, Clamped };

 public:
  CubicSpline() = default;
  explicit CubicSpline(const std::vector<Knot> &knots);
  CubicSpline(double fp0, double fpn, const std::vector<Knot> &knots);

  std::vector<Knot> GetKnots() const { return knots_; }

  /* default: use knots_ to interpolate spline */
  // interpolate with natual boundary condition
  void Interpolate(const std::vector<Knot> &knots = {});
  // interpolate with clamped boundary condition
  void Interpolate(double fp0, double fpn, const std::vector<Knot> &knots = {});
  double Evaluate(double x, uint32_t derivative = 0) const;

  Type GetInterpolationType() const { return type_; }

  Eigen::MatrixXd GetCoefficients() const { return coefficients_; }

 private:
  Type type_;
  std::vector<Knot> knots_;
  Eigen::MatrixXd coefficients_;
};

#ifdef ENABLE_VISUAL
void DrawCubicSpline(quickviz::CvCanvas &canvas, const CubicSpline &spline,
                     double step = 0.01,
                     cv::Scalar ln_color = quickviz::CvColors::blue_color,
                     int32_t thickness = 1);
#endif
}  // namespace xmotion

#endif /* CUBIC_SPLINE_HPP */
