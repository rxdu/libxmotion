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

#include "lightviz/details/cartesian_canvas.hpp"

#include "geometry/polygon.hpp"
#include "geometry/cspline.hpp"
#include "geometry/parametric_curve.hpp"
#include "decomp/curvilinear_grid.hpp"
#include "canvas/cv_draw.hpp"

namespace librav
{
class GeometryDraw
{
  public:
    GeometryDraw(CartesianCanvas &canvas) : canvas_(canvas){};

    // curve
    void DrawPolyline(const Polyline &polyline, bool show_dot = false, cv::Scalar ln_color = CvDrawColors::blue_color, int32_t ln_width = 1);
    void DrawCubicSpline(const CSpline &spline, double step = 0.01, cv::Scalar ln_color = CvDrawColors::blue_color, int32_t ln_width = 1);
    void DrawParametricCurve(const ParametricCurve &pcurve, double step = 0.1, cv::Scalar ln_color = CvDrawColors::blue_color, int32_t ln_width = 1);

    // geometric grid
    void DrawCurvilinearGrid(const CurvilinearGrid &grid, double step = 0.1, bool show_center = false, cv::Scalar ln_color = CvDrawColors::lime_color, int32_t ln_width = 1);

    // polygon
    void DrawPolygon(const Polygon &polygon, bool show_dot = false, cv::Scalar ln_color = CvDrawColors::blue_color, int32_t ln_width = 1);
    void DrawFilledPolygon(const Polygon &polygon, bool show_dot = false, cv::Scalar fill_color = CvDrawColors::aoi_color, cv::Scalar ln_color = CvDrawColors::blue_color, int32_t ln_width = 1);

    // annotations
    void DrawPolygonDirection(const Polygon &polygon, cv::Scalar ln_color = CvDrawColors::blue_color, int32_t ln_width = 1);
    void WritePointPosition(const std::vector<SimplePoint> &points);

    // distribution
    void DrawDistribution(double cx, double cy, double xspan, double yspan, std::function<double(double, double)> dist_fun);

  private:
    // internal parameters
    CartesianCanvas &canvas_;
};
} // namespace librav

#endif /* GEOMETRY_DRAW_HPP */
