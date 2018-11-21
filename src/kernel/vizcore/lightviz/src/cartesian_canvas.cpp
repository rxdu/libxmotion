/* 
 * cartesian_canvas.cpp
 * 
 * Created on: Oct 25, 2018 11:46
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/details/cartesian_canvas.hpp"

#include <cassert>

#include "lightviz/details/matrix_draw.hpp"

using namespace librav;
using namespace LightViz;
using namespace CvDraw;

void CartesianCanvas::SetupCanvas(double xmin, double xmax, double ymin, double ymax, cv::Scalar bg_color)
{
    assert(xmax > xmin && ymax > ymin);

    xmin_ = xmin;
    xmax_ = xmax;
    ymin_ = ymin;
    ymax_ = ymax;

    // create canvas
    xspan_ = xmax - xmin;
    yspan_ = ymax - ymin;
    canvas_size_x_ = xspan_ * ppu_;
    canvas_size_y_ = yspan_ * ppu_;

    cv::Mat canvas(canvas_size_y_, canvas_size_x_, CV_8UC3, bg_color);
    paint_area = canvas;

    // std::cout << "canvas size: " << canvas_size_x_ << " , " << canvas_size_y_ << std::endl;
}

void CartesianCanvas::SetupCanvas(int32_t xmax, int32_t ymax, cv::Scalar bg_color)
{
    canvas_size_x_ = xmax;
    canvas_size_y_ = ymax;

    cv::Mat canvas(canvas_size_y_, canvas_size_x_, CV_8UC3, bg_color);
    paint_area = canvas;

    // std::cout << "canvas size: " << canvas_size_x_ << " , " << canvas_size_y_ << std::endl;
}

SimplePoint CartesianCanvas::ConvertCartisianToPixel(double xi, double yi)
{
    int32_t x = (xi - xmin_) / xspan_ * canvas_size_x_;
    int32_t y = canvas_size_y_ - (yi - ymin_) / yspan_ * canvas_size_y_;

    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= canvas_size_x_)
        x = canvas_size_x_ - 1;
    if (y >= canvas_size_y_)
        y = canvas_size_y_ - 1;
    return SimplePoint(x, y);
}