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

cv::Mat CartesianCanvas::GetROIofPaintArea(double cx, double cy, double xspan, double yspan)
{
    double rxmin = cx - xspan / 2.0;
    double rxmax = cx + xspan / 2.0;
    double rymin = cy - yspan / 2.0;
    double rymax = cy + yspan / 2.0;

    // std::cout << rxmin << " , " << rxmax << " ; " << rymin << " , " << rymax << std::endl;

    SimplePoint tl_pt = ConvertCartisianToPixel(rxmin, rymax);
    SimplePoint br_pt = ConvertCartisianToPixel(rxmax, rymin);

    // std::cout << "tl: " << tl_pt.x << " , " << tl_pt.y << std::endl;
    // std::cout << "br: " << br_pt.x << " , " << br_pt.y << std::endl;

    int width = br_pt.x - tl_pt.x;
    int height = br_pt.y - tl_pt.y;

    double aspct_ratio = canvas_size_y_ / canvas_size_x_;
    int height2 = static_cast<int>(width * aspct_ratio);

    // std::cout << "width: " << width << " , height: " << height << std::endl;

    cv::Mat roiImage = paint_area(cv::Rect(tl_pt.x, tl_pt.y, width, height2));

    cv::Mat output;
    roiImage.copyTo(output);

    return output;
}

cv::Mat CartesianCanvas::GetROIofPaintArea(double cx, double cy, double ratio)
{
    double xspan = (xmax_ - xmin_) * ratio;
    double yspan = (ymax_ - ymin_) * ratio;

    double rxmin = cx - xspan / 2.0;
    double rxmax = cx + xspan / 2.0;
    double rymin = cy - yspan / 2.0;
    double rymax = cy + yspan / 2.0;

    // std::cout << rxmin << " , " << rxmax << " ; " << rymin << " , " << rymax << std::endl;

    SimplePoint tl_pt = ConvertCartisianToPixel(rxmin, rymax);
    SimplePoint br_pt = ConvertCartisianToPixel(rxmax, rymin);

    // std::cout << "tl: " << tl_pt.x << " , " << tl_pt.y << std::endl;
    // std::cout << "br: " << br_pt.x << " , " << br_pt.y << std::endl;

    int width = br_pt.x - tl_pt.x;
    int height = br_pt.y - tl_pt.y;

    // std::cout << "width: " << width << " , height: " << height << std::endl;

    if (tl_pt.x + width > canvas_size_x_)
        tl_pt.x = canvas_size_x_ - width - 1;
    if (tl_pt.y + height > canvas_size_y_)
        tl_pt.y = canvas_size_y_ - height - 1;

    cv::Mat roiImage = paint_area(cv::Rect(tl_pt.x, tl_pt.y, width, height));

    cv::Mat output;
    roiImage.copyTo(output);

    return output;
}