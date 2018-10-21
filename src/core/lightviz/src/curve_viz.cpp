/* 
 * curve_viz.cpp
 * 
 * Created on: Oct 19, 2018 11:21
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/curve_viz.hpp"

#include <cmath>

#include "lightviz/details/geometric_draw.hpp"

using namespace librav;

void LightViz::ShowCubicSpline(const CSpline &spline, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(-15, 15, -10, 10);
    canvas = gdraw.DrawCubicSpline(canvas, spline, step);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowCubicSpline(const std::vector<CSpline> &splines, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
}

void LightViz::ShowCubicSplinePosition(const std::vector<CSpline> &splines, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
}

void LightViz::ShowParametricCurve(const ParametricCurve &pcurve, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(-15, 15, -10, 10);
    canvas = gdraw.DrawParametricCurve(canvas, pcurve, step);

    ShowImage(canvas, window_name, save_img);
}
