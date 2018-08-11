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

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

// #include "decomp/dense_grid.hpp"
#include "polygon/polygon.hpp"
#include "lightviz/cv_draw.hpp"

namespace librav
{
struct GeometryDraw
{
    GeometryDraw(int32_t ppu) : pixel_per_meter(ppu) {}

    cv::Mat CreateCanvas(double xmin, double xmax, double ymin, double ymax);
    cv::Mat CreateCanvasWithMatrixColorMap(const Eigen::MatrixXd &matrix, int32_t ppu);

    cv::Mat DrawPolyline(cv::Mat canvas, const Polyline &polyline, bool show_dot = false, cv::Scalar ln_color = LVColors::blue_color);
    cv::Mat DrawPolygon(cv::Mat canvas, const Polygon &polygon, bool show_dot = false, cv::Scalar ln_color = LVColors::blue_color);

    SimplePoint ConvertCartisianToPixel(double xi, double yi);

    int32_t pixel_per_meter = 100;
    double xmin_;
    double xmax_;
    double xspan_;
    double yspan_;
    double ymin_;
    double ymax_;
    int32_t canvas_size_x_;
    int32_t canvas_size_y_;
};
} // namespace librav

#endif /* GEOMETRY_DRAW_HPP */
