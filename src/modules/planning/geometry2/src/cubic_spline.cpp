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
#include <algorithm>
#include <cassert>

namespace robotnav {
CubicSpline::CubicSpline(const std::vector<Knot> &knots) : knots_(knots) {
  Interpolate();
}

void CubicSpline::Interpolate(const std::vector<Knot> &knots) {
  // replace existing knots if a non-empty set of Knots is passed in
  if (!knots.empty()) knots_ = knots;

  if (knots_.empty()) return;

  // interpolation
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
