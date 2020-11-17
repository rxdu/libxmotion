/* 
 * curve_viz.hpp
 * 
 * Created on: Oct 19, 2018 11:21
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef CURVE_VIZ_HPP
#define CURVE_VIZ_HPP

#include <vector>
#include <string>

#include "geometry/cspline.hpp"
#include "geometry/parametric_curve.hpp"

namespace rnav
{
namespace LightViz {
void ShowCubicSpline(const CSpline &spline, double step = 0.01, int32_t pixel_per_unit = 10, std::string window_name = "Spline Image", bool save_img = false);
void ShowCubicSpline(const std::vector<CSpline> &splines, double step = 0.01, int32_t pixel_per_unit = 10, std::string window_name = "Spline Image", bool save_img = false);
void ShowCubicSplinePosition(const std::vector<CSpline> &splines, double step = 0.01, int32_t pixel_per_unit = 10, std::string window_name = "Spline Image", bool save_img = false);

void ShowParametricCurve(const ParametricCurve &pcurve, double step = 0.01, int32_t pixel_per_unit = 10, std::string window_name = "Spline Image", bool save_img = false);
} // namespace LightViz
} // namespace rnav

#endif /* CURVE_VIZ_HPP */
