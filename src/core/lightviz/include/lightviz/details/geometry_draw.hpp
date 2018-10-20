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

// #include "decomp/dense_grid.hpp"
#include "geometry/polygon.hpp"
#include "geometry/cspline.hpp"
#include "geometry/parametric_curve.hpp"
#include "lightviz/cv_draw.hpp"

namespace librav
{
struct GeometryDraw
{
    GeometryDraw(int32_t ppu) : pixel_per_meter(ppu) {}

    cv::Mat CreateCanvas(double xmin, double xmax, double ymin, double ymax, cv::Scalar bg_color = LVColors::bg_color);
    cv::Mat CreateCanvasWithMatrixColorMap(const Eigen::MatrixXd &matrix, int32_t ppu);

    cv::Mat DrawPolyline(cv::Mat canvas, const Polyline &polyline, bool show_dot = false, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);
    cv::Mat DrawCubicSpline(cv::Mat canvas, const CSpline &spline, double step = 0.01, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);
    cv::Mat DrawParametricCurve(cv::Mat canvas, const ParametricCurve& pcurve, double step = 0.1, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);

    cv::Mat DrawPolygon(cv::Mat canvas, const Polygon &polygon, bool show_dot = false, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);
    cv::Mat DrawFilledPolygon(cv::Mat canvas, const Polygon &polygon, bool show_dot = false, cv::Scalar fill_color = LVColors::aoi_color, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);

    cv::Mat DrawPolygonDirection(cv::Mat canvas, const Polygon &polygon, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);
    cv::Mat WritePointPosition(cv::Mat canvas, const std::vector<SimplePoint> &points);

    cv::Mat DrawDistribution(cv::Mat canvas, double cx, double cy, double xspan, double yspan, std::function<double(double, double)> dist_fun);

    SimplePoint ConvertCartisianToPixel(double xi, double yi);

    int32_t pixel_per_meter = 10;
    double xmin_;
    double xmax_;
    double ymin_;
    double ymax_;
    double xspan_;
    double yspan_;
    int32_t canvas_size_x_;
    int32_t canvas_size_y_;
};
} // namespace librav

#endif /* GEOMETRY_DRAW_HPP */
