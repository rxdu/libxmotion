/* 
 * parametric_curve.hpp
 * 
 * Created on: Oct 20, 2018 08:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef PARAMETRIC_CURVE_HPP
#define PARAMETRIC_CURVE_HPP

#include "geometry/polyline.hpp"
#include "geometry/cspline.hpp"
#include "geometry/simple_point.hpp"

namespace librav
{
// 2D parametric curve: x, y 
// each dimension is represented as a cubic spline
class ParametricCurve
{
public:
  ParametricCurve() = default;
  explicit ParametricCurve(Polyline center_polyline);
  ParametricCurve(CSpline xspline, CSpline yspline, double sf);
  ~ParametricCurve() = default;

  ParametricCurve(const ParametricCurve &other) = default;
  ParametricCurve &operator=(const ParametricCurve &other) = default;
  ParametricCurve(ParametricCurve &&other) = default;
  ParametricCurve &operator=(ParametricCurve &&other) = default;

  SimplePoint Evaluate(double s) const;
  SimplePoint Evaluate(double s, int32_t derivative) const;

  double GetTotalLength() const { return total_length_; }
  CSpline GetXSpline() const { return x_spline_; }
  CSpline GetYSpline() const { return y_spline_; }

private:
  double total_length_;

  CSpline x_spline_;
  CSpline y_spline_;

  Polyline polyline_;
};

namespace CurveFitting
{
ParametricCurve FitApproximateLengthCurve(Polyline polyline);
}
} // namespace librav

#endif /* PARAMETRIC_CURVE_HPP */
