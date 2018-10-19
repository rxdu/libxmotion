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

namespace librav
{
class PathCurve
{
public:
  PathCurve() = default;
  explicit PathCurve(Polyline center_polyline);
  ~PathCurve() = default;

  PathCurve(const PathCurve &other) = default;
  PathCurve &operator=(const PathCurve &other) = default;
  PathCurve(PathCurve &&other) = default;
  PathCurve &operator=(PathCurve &&other) = default;

private:
  CSpline x_spline_;
  CSpline y_spline_;

  Polyline polyline_;
};
} // namespace librav

#endif /* PATH_CURVE_HPP */
