/*
 * parametric_curve.hpp
 *
 * Created on: Oct 20, 2018 08:44
 * Description: parametric curve with each dimension represented
 *              as a cubic spline
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef PARAMETRIC_CURVE_HPP
#define PARAMETRIC_CURVE_HPP

#include <vector>

#include "geometry/polyline.hpp"
#include "geometry/cubic_spline.hpp"
#include "geometry/simple_point.hpp"

#ifdef ENABLE_VISUAL
#include "cvdraw/cvdraw.hpp"
#endif

namespace xmotion {
// 2D parametric curve: x, y
// each dimension is represented as a cubic spline
class ParametricCurve {
 public:
  ParametricCurve() = default;
  explicit ParametricCurve(Polyline center_polyline);
  ParametricCurve(CubicSpline xspline, CubicSpline yspline, double sf);

  SimplePoint2 Evaluate(double s, int32_t derivative = 0) const;

  void GetPositionVector(double s, double &x, double &y) const;
  void GetTangentVector(double s, double &x, double &y) const;

  double GetLength() const { return total_length_; }
  CubicSpline GetXSpline() const { return x_spline_; }
  CubicSpline GetYSpline() const { return y_spline_; }

 private:
  Polyline polyline_;
  double total_length_;

  CubicSpline x_spline_;
  CubicSpline y_spline_;
};

namespace CurveFitting {
ParametricCurve FitApproximateLengthCurve(Polyline polyline);
ParametricCurve FitTimedCurve(std::vector<double> x, std::vector<double> y,
                              std::vector<double> t);
}  // namespace CurveFitting

#ifdef ENABLE_VISUAL
void DrawParametricCurve(quickviz::CvCanvas &canvas, const ParametricCurve &pcurve,
                         double step = 0.1,
                         cv::Scalar ln_color = quickviz::CvColors::blue_color,
                         int32_t thickness = 1);
#endif
}  // namespace xmotion

#endif /* PARAMETRIC_CURVE_HPP */
