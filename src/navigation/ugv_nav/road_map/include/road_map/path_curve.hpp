/* 
 * path_curve.hpp
 * 
 * Created on: Oct 19, 2018 11:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef PATH_CURVE_HPP
#define PATH_CURVE_HPP

#include "geometry/polyline.hpp"
#include "geometry/cspline.hpp"
#include "geometry/simple_point.hpp"

namespace librav
{
class PathCurve
{
public:
  PathCurve() = default;
  explicit PathCurve(Polyline center_polyline);
  PathCurve(CSpline xspline, CSpline yspline, double sf);
  ~PathCurve() = default;

  PathCurve(const PathCurve &other) = default;
  PathCurve &operator=(const PathCurve &other) = default;
  PathCurve(PathCurve &&other) = default;
  PathCurve &operator=(PathCurve &&other) = default;

  double GetTotalLength() const { return total_length_; }
  CSpline GetXSpline() const { return x_spline_; }
  CSpline GetYSpline() const { return y_spline_; }

  SimplePoint Evaluate(double s) const;
  SimplePoint Evaluate(double s, int32_t derivative) const;

private:
  double total_length_;
  CSpline x_spline_;
  CSpline y_spline_;

  Polyline polyline_;
};

namespace CurveFitting
{
PathCurve FitApproximateLengthCurve(Polyline polyline);
}
} // namespace librav

#endif /* PATH_CURVE_HPP */
