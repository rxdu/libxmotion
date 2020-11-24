/*
 * cubic_spline.hpp
 *
 * Created on: Oct 13, 2018 11:02
 * Description: an implementation of Clamped Cubic Spline
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include <vector>

#include "geometry/simple_point.hpp"

namespace rnav {
class CubicSpline {
 public:
  using Knot = SimplePoint2;

 public:
  CubicSpline() = default;
  explicit CubicSpline(const std::vector<Knot> &knots);
  ~CubicSpline() = default;

  // default: use knots_ to interpolate spline
  void Interpolate(const std::vector<Knot> &knots = {});
  double Evaluate(double x, uint32_t derivative = 0) const;

  std::vector<Knot> GetKnots() const { return knots_; }

 private:
  std::vector<Knot> knots_;
};
}  // namespace rnav

#endif /* CUBIC_SPLINE_HPP */
