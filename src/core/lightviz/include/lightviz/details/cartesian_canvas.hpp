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

#include "lightviz/cv_draw.hpp"
#include "geometry/simple_point.hpp"

namespace librav
{
struct CartesianCanvas
{
    CartesianCanvas(int32_t ppu) : pixel_per_meter(ppu) {}

    // create canvas
    void CreateCanvas(double xmin, double xmax, double ymin, double ymax, cv::Scalar bg_color = LVColors::bg_color);

    // coordinate conversion
    SimplePoint ConvertCartisianToPixel(double xi, double yi);

    cv::Mat paint_area;

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

#endif /* CARTESIAN_CANVAS_HPP */
