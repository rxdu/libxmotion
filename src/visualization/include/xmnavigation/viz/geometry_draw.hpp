/*
 * geometry_draw.hpp
 *
 * Drawing functions for the geometry primitives (extracted from the
 * geometry module so algorithm libraries stay visualization-free).
 *
 * Copyright (c) 2018-2026 Ruixiang Du (rdu)
 */

#ifndef XMNAVIGATION_VIZ_GEOMETRY_DRAW_HPP
#define XMNAVIGATION_VIZ_GEOMETRY_DRAW_HPP

#include "cvdraw/cvdraw.hpp"

#include "xmnavigation/geometry/polyline.hpp"
#include "xmnavigation/geometry/cubic_spline.hpp"
#include "xmnavigation/geometry/parametric_curve.hpp"
#include "xmnavigation/geometry/polygon.hpp"

namespace xmotion {
void DrawPolyline(quickviz::CvCanvas& canvas, const Polyline& polyline,
                  bool show_dot = false,
                  cv::Scalar ln_color = quickviz::CvColors::blue_color,
                  int32_t thickness = 1);

void DrawCubicSpline(quickviz::CvCanvas &canvas, const CubicSpline &spline,
                     double step = 0.01,
                     cv::Scalar ln_color = quickviz::CvColors::blue_color,
                     int32_t thickness = 1);

void DrawParametricCurve(quickviz::CvCanvas &canvas, const ParametricCurve &pcurve,
                         double step = 0.1,
                         cv::Scalar ln_color = quickviz::CvColors::blue_color,
                         int32_t thickness = 1);

void DrawPolygon(quickviz::CvCanvas &canvas, const Polygon &polygon,
                 bool show_dot = false,
                 cv::Scalar ln_color = quickviz::CvColors::blue_color,
                 int32_t thickness = 1);
void FillPolygon(quickviz::CvCanvas &canvas, const Polygon &polygon,
                 bool show_dot = false,
                 cv::Scalar fill_color = quickviz::CvColors::aoi_color,
                 cv::Scalar ln_color = quickviz::CvColors::blue_color,
                 int32_t thickness = 1);
}  // namespace xmotion

#endif  // XMNAVIGATION_VIZ_GEOMETRY_DRAW_HPP
