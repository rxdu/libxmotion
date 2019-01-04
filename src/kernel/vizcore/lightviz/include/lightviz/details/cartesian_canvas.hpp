/* 
 * cartesian_canvas.hpp
 * 
 * Created on: Oct 25, 2018 11:45
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CARTESIAN_CANVAS_HPP
#define CARTESIAN_CANVAS_HPP

#include <cstdint>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "canvas/cv_draw.hpp"
#include "geometry/simple_point.hpp"

namespace librav
{
struct CartesianCanvas
{
    CartesianCanvas(int32_t ppu) : ppu_(ppu) {}

    // setup canvas
    void SetupCanvas(double xmin, double xmax, double ymin, double ymax, cv::Scalar bg_color = CvDrawColors::bg_color);
    void SetupCanvas(int32_t xmax, int32_t ymax, cv::Scalar bg_color = CvDrawColors::bg_color);
    
    void ClearCanvas(cv::Scalar bg_color = CvDrawColors::bg_color);

    // coordinate conversion
    SimplePoint ConvertCartisianToPixel(double xi, double yi);
    cv::Mat GetROIofPaintArea(double cx, double cy, double xspan, double yspan);
    cv::Mat GetROIofPaintArea(double cx, double cy, double ratio);

    // the place for painting
    cv::Mat paint_area;

    // size parameters
    int32_t ppu_ = 10;
    double xmin_ = 0.0;
    double xmax_ = 80.0;
    double ymin_ = 0.0;
    double ymax_ = 60.0;
    double xspan_ = 80.0;
    double yspan_ = 60.0;
    int32_t canvas_size_x_ = 800;
    int32_t canvas_size_y_ = 600;
};
} // namespace librav

#endif /* CARTESIAN_CANVAS_HPP */
