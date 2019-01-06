/* 
 * geometry_draw.hpp
 * 
 * Created on: Aug 10, 2018 09:16
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GEOMETRY_DRAW_HPP
#define GEOMETRY_DRAW_HPP

#include <cstdint>
#include <functional>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "geometry/polygon.hpp"
#include "geometry/cspline.hpp"
#include "geometry/parametric_curve.hpp"
#include "decomp/curvilinear_grid.hpp"

#include "cvdraw/cvdraw.hpp"
#include "coreviz/matrix_draw.hpp"

namespace librav
{
struct GeometryDraw
{
    // curve
    static void DrawPolyline(CvCanvas &canvas, const Polyline &polyline, bool show_dot = false, cv::Scalar ln_color = CvColors::blue_color, int32_t thickness = 1);
    static void DrawCubicSpline(CvCanvas &canvas, const CSpline &spline, double step = 0.01, cv::Scalar ln_color = CvColors::blue_color, int32_t thickness = 1);
    static void DrawParametricCurve(CvCanvas &canvas, const ParametricCurve &pcurve, double step = 0.1, cv::Scalar ln_color = CvColors::blue_color, int32_t thickness = 1);

    // polygon
    static void DrawLabelPoint(CvCanvas &canvas, double x, double y, cv::Scalar ln_color = CvColors::blue_color, int32_t thickness = 1);
    static void DrawPolygon(CvCanvas &canvas, const Polygon &polygon, bool show_dot = false, cv::Scalar ln_color = CvColors::blue_color, int32_t thickness = 1);
    static void FillPolygon(CvCanvas &canvas, const Polygon &polygon, bool show_dot = false, cv::Scalar fill_color = CvColors::aoi_color, cv::Scalar ln_color = CvColors::blue_color, int32_t thickness = 1);

    // annotations
    static void DrawPolygonDirection(CvCanvas &canvas, const Polygon &polygon, cv::Scalar ln_color = CvColors::blue_color, int32_t thickness = 1);
    static void WriteTextAtPosition(CvCanvas &canvas, std::string txt, SimplePoint pt);
    static void WritePointPosition(CvCanvas &canvas, const std::vector<SimplePoint> &points);

    // distribution
    static void DrawDistribution(CvCanvas &canvas, double cx, double cy, double xspan, double yspan, std::function<double(double, double)> dist_fun);
};
} // namespace librav

#endif /* GEOMETRY_DRAW_HPP */
