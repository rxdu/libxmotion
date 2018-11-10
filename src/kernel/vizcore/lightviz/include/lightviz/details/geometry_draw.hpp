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
#include "lightviz/details/matrix_draw.hpp"

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

    // polygon
    void DrawPolygon(const Polygon &polygon, bool show_dot = false, cv::Scalar ln_color = CvDrawColors::blue_color, int32_t ln_width = 1);
    void DrawFilledPolygon(const Polygon &polygon, bool show_dot = false, cv::Scalar fill_color = CvDrawColors::aoi_color, cv::Scalar ln_color = CvDrawColors::blue_color, int32_t ln_width = 1);

    // annotations
    void DrawPolygonDirection(const Polygon &polygon, cv::Scalar ln_color = CvDrawColors::blue_color, int32_t ln_width = 1);
    void WriteTextAtPosition(std::string txt, SimplePoint pt);
    void WritePointPosition(const std::vector<SimplePoint> &points);

    // distribution
    void DrawDistribution(double cx, double cy, double xspan, double yspan, std::function<double(double, double)> dist_fun);
    template <typename T>
    void DrawDistributionFast(double cx, double cy, double xspan, double yspan, T dist_fun)
    {
        assert(cx >= canvas_.xmin_ && cx < canvas_.xmax_ && cy >= canvas_.ymin_ && cy < canvas_.ymax_);

        // distributions coverage x/y limits
        double dxmin = cx - xspan / 2.0;
        double dxmax = cx + xspan / 2.0;
        double dymin = cy - yspan / 2.0;
        double dymax = cy + yspan / 2.0;

        // crop distribution to canvas area
        if (dxmin < canvas_.xmin_)
            dxmin = canvas_.xmin_;
        if (dxmax > canvas_.xmax_)
            dxmax = canvas_.xmax_;
        if (dymin < canvas_.ymin_)
            dymin = canvas_.ymin_;
        if (dymax > canvas_.ymax_)
            dymax = canvas_.ymax_;

        double dxspan = dxmax - dxmin;
        double dyspan = dymax - dymin;
        int32_t x_size = dxspan * canvas_.ppu_;
        int32_t y_size = dyspan * canvas_.ppu_;

        Eigen::MatrixXd threat_matrix = Eigen::MatrixXd::Zero(y_size, x_size);
        int32_t meter_per_pixel = 1 / canvas_.ppu_;

        std::cout << "started applying function" << std::endl;
        double ppu = canvas_.ppu_;
        for (int32_t i = 0; i < x_size; ++i)
            for (int32_t j = 0; j < y_size; ++j)
            {
                // convert to cartisian coordinate
                // double x = dxmin + i / ppu;
                // double y = dymin + j / ppu;

                threat_matrix(j, i) = dist_fun(dxmin + i / ppu, dymin + j / ppu);
            }
        std::cout << "finished applying function" << std::endl;

        cv::Mat threat_vis = LightViz::CreateColorMapFromEigenMatrix(threat_matrix, true);

        // merge threat distribution to canvas
        auto top_left_pixel = canvas_.ConvertCartisianToPixel(dxmin, dymax); // y inverted in cartesian coordinate
        threat_vis.copyTo(canvas_.paint_area(cv::Rect(top_left_pixel.x, top_left_pixel.y, threat_vis.cols, threat_vis.rows)));
    }

    cv::Mat GetPaintArea() { return canvas_.paint_area; }

  private:
    // internal parameters
    CartesianCanvas &canvas_;
};
} // namespace librav

#endif /* GEOMETRY_DRAW_HPP */
