/* 
 * cv_canvas.cpp
 * 
 * Created on: Jan 04, 2019 11:09
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "cvdraw/cv_canvas.hpp"

namespace librav
{
using namespace cv;

CvCanvas::CvCanvas(cv::Mat background, std::string name) : paint_area_(background),
                                                           canvas_name_(name)
{
    canvas_size_x_ = background.cols;
    canvas_size_y_ = background.rows;
}

CvCanvas::CvCanvas(int32_t px, int32_t py, cv::Scalar bg_color, std::string name) : canvas_size_x_(px),
                                                                                    canvas_size_y_(py),
                                                                                    bg_color_(bg_color),
                                                                                    canvas_name_(name)
{
    InitCanvas();
}

void CvCanvas::InitCanvas()
{
    assert(canvas_size_x_ > 0 && canvas_size_y_ > 0);
    cv::Mat canvas(canvas_size_y_, canvas_size_x_, CV_8UC3, bg_color_);
    paint_area_ = canvas;
}

void CvCanvas::SetupCanvas(double xmin, double xmax, double ymin, double ymax, cv::Scalar bg_color)
{
    assert(xmax > xmin && ymax > ymin);

    xmin_ = xmin;
    xmax_ = xmax;
    ymin_ = ymin;
    ymax_ = ymax;

    xspan_ = xmax - xmin;
    yspan_ = ymax - ymin;
    canvas_size_x_ = xspan_ * ppu_;
    canvas_size_y_ = yspan_ * ppu_;

    bg_color_ = bg_color;

    // create canvas
    InitCanvas();
}

void CvCanvas::ClearCanvas(cv::Scalar bg_color)
{
    paint_area_.release();
    cv::Mat new_canvas(canvas_size_y_, canvas_size_x_, CV_8UC3, bg_color);
    paint_area_ = new_canvas;
}

void CvCanvas::Save(std::string filename)
{
    imwrite(filename + ".png", paint_area_);
}

void CvCanvas::Show(bool use_img_size)
{
    int flags = WINDOW_NORMAL;
    if (use_img_size)
        flags = WINDOW_AUTOSIZE;
    namedWindow(canvas_name_, flags | WINDOW_KEEPRATIO | WINDOW_GUI_EXPANDED);
    imshow(canvas_name_, paint_area_);

    waitKey(0);
    destroyWindow(canvas_name_);
}

void CvCanvas::ShowFrame(int32_t frame_period_ms)
{
    imshow(canvas_name_, paint_area_);
    waitKey(frame_period_ms);
}
} // namespace librav
