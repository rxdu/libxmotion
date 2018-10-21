/* 
 * geometric_draw.hpp
 * 
 * Created on: Aug 10, 2018 09:16
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GEOMETRIC_DRAW_HPP
#define GEOMETRIC_DRAW_HPP

#include <cstdint>
#include <functional>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "geometry/polygon.hpp"
#include "geometry/cspline.hpp"
#include "geometry/parametric_curve.hpp"
#include "decomp/curvilinear_grid.hpp"
#include "lightviz/cv_draw.hpp"

namespace librav
{
struct GeometryDraw
{
    GeometryDraw(int32_t ppu) : pixel_per_meter(ppu) {}

    // canvas
    cv::Mat CreateCanvas(double xmin, double xmax, double ymin, double ymax, cv::Scalar bg_color = LVColors::bg_color);
    cv::Mat CreateCanvasWithMatrixColorMap(const Eigen::MatrixXd &matrix, int32_t ppu);

    // curve
    cv::Mat DrawPolyline(cv::Mat canvas, const Polyline &polyline, bool show_dot = false, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);
    cv::Mat DrawCubicSpline(cv::Mat canvas, const CSpline &spline, double step = 0.01, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);
    cv::Mat DrawParametricCurve(cv::Mat canvas, const ParametricCurve &pcurve, double step = 0.1, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);

    // geometric grid
    cv::Mat DrawCurvilinearGrid(cv::Mat canvas, const CurvilinearGrid &grid, double step = 0.1, bool show_center = false, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);

    // polygon
    cv::Mat DrawPolygon(cv::Mat canvas, const Polygon &polygon, bool show_dot = false, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);
    cv::Mat DrawFilledPolygon(cv::Mat canvas, const Polygon &polygon, bool show_dot = false, cv::Scalar fill_color = LVColors::aoi_color, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);

    // annotations
    cv::Mat DrawPolygonDirection(cv::Mat canvas, const Polygon &polygon, cv::Scalar ln_color = LVColors::blue_color, int32_t ln_width = 1);
    cv::Mat WritePointPosition(cv::Mat canvas, const std::vector<SimplePoint> &points);

    // distribution
    cv::Mat DrawDistribution(cv::Mat canvas, double cx, double cy, double xspan, double yspan, std::function<double(double, double)> dist_fun);

    // coordinate conversion
    SimplePoint ConvertCartisianToPixel(double xi, double yi);

    // internal parameters
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

#endif /* GEOMETRIC_DRAW_HPP */
