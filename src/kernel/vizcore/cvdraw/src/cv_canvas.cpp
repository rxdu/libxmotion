/* 
 * cv_canvas.cpp
 * 
 * Created on: Jan 04, 2019 11:09
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "cvdraw/cv_canvas.hpp"

#include <cassert>

namespace librav
{
using namespace cv;

CvCanvas::CvCanvas(cv::Mat background, std::string name) : init_paint_(background),
                                                           canvas_name_(name)
{
    canvas_size_x_ = background.cols;
    canvas_size_y_ = background.rows;
    img_bg_mode_ = true;

    InitCanvas();
}

CvCanvas::CvCanvas(int32_t px, int32_t py, cv::Scalar bg_color, std::string name) : canvas_size_x_(px),
                                                                                    canvas_size_y_(py),
                                                                                    bg_color_(bg_color),
                                                                                    canvas_name_(name)
{
    InitCanvas();
}

CvCanvas::CvCanvas(int32_t ppu, cv::Scalar bg_color, std::string name) : ppu_(ppu),
                                                                         bg_color_(bg_color),
                                                                         canvas_name_(name)
{
    // init with default values of canvas_size_x_ and canvas_size_y_
    InitCanvas();

    // use geometry mode
    draw_mode_ = DrawMode::Geometry;
}

void CvCanvas::InitCanvas()
{
    assert(canvas_size_x_ > 0 && canvas_size_y_ > 0);

    if (!img_bg_mode_)
    {
        if (!init_paint_.empty())
            init_paint_.release();
        cv::Mat canvas(canvas_size_y_, canvas_size_x_, CV_8UC3, bg_color_);
        init_paint_ = canvas;
    }

    paint_area_ = init_paint_.clone();
}

void CvCanvas::Resize(double px, double py)
{
    assert(px > 0 && py > 0);

    canvas_size_x_ = px;
    canvas_size_y_ = py;

    InitCanvas();
}

void CvCanvas::Resize(double xmin, double xmax, double ymin, double ymax)
{
    assert(xmax > xmin && ymax > ymin);

    if (img_bg_mode_)
        return;

    xmin_ = xmin;
    xmax_ = xmax;
    ymin_ = ymin;
    ymax_ = ymax;

    xspan_ = xmax - xmin;
    yspan_ = ymax - ymin;

    canvas_size_x_ = xspan_ * ppu_;
    canvas_size_y_ = yspan_ * ppu_;

    InitCanvas();
}

void CvCanvas::Clear()
{
    paint_area_.release();
    paint_area_ = init_paint_.clone();
}

void CvCanvas::FillBackgroundColor(cv::Scalar bg_color)
{
    bg_color_ = bg_color;

    InitCanvas();
}

// cv::Point CvCanvas::ConvertCvPointToPixel(CPoint pt)
// {
//     if (draw_mode_ == DrawMode::Raster)
//         return cv::Point(pt.x, pt.y);
//     else if (draw_mode_ == DrawMode::Geometry)
//         return ConvertGeometryPointToPixel(pt.x, pt.y);
//     else // if (draw_mode_ == DrawMode::Vector)
//         return ConvertNormalizedPointToPixel(pt.x, pt.y);
// }

// void CvCanvas::ClampToCanvasSize(int32_t &x, int32_t y)
// {
//     if (x < 0)
//         x = 0;
//     if (y < 0)
//         y = 0;
//     if (x >= canvas_size_x_)
//         x = canvas_size_x_ - 1;
//     if (y >= canvas_size_y_)
//         y = canvas_size_y_ - 1;
// }

cv::Point CvCanvas::ConvertGeometryPointToPixel(double xi, double yi)
{
    int32_t x = (xi - xmin_) / xspan_ * canvas_size_x_;
    int32_t y = canvas_size_y_ - (yi - ymin_) / yspan_ * canvas_size_y_;

    ClampToCanvasSize(x, y);

    return cv::Point(x, y);
}

cv::Point CvCanvas::ConvertNormalizedPointToPixel(double xi, double yi)
{
    int32_t x = xi * canvas_size_x_;
    int32_t y = yi * canvas_size_y_;

    ClampToCanvasSize(x, y);

    return cv::Point(x, y);
}

void CvCanvas::SavePaint(std::string filename)
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

void CvCanvas::DrawPoint(CPoint center, int radius, const cv::Scalar &color, int line_type)
{
    cv::Point cv_center = ConvertCvPointToPixel(center);
    circle(paint_area_, cv_center, radius, color, CV_FILLED, line_type);
}

void CvCanvas::DrawCircle(CPoint center, int radius, const cv::Scalar &color, int thickness, int line_type)
{
    cv::Point cv_center = ConvertCvPointToPixel(center);
    circle(paint_area_, cv_center, radius, color, thickness, line_type);
}

void CvCanvas::DrawLine(CPoint pt1, CPoint pt2, const cv::Scalar &color, int thickness, int line_type)
{
    cv::Point cv_pt1 = ConvertCvPointToPixel(pt1);
    cv::Point cv_pt2 = ConvertCvPointToPixel(pt2);
    line(paint_area_, cv_pt1, cv_pt2, color, thickness, line_type);
}

void CvCanvas::DrawArrowedLine(CPoint pt1, CPoint pt2, double tip_length, const cv::Scalar &color, int thickness, int line_type)
{
    cv::Point cv_pt1 = ConvertCvPointToPixel(pt1);
    cv::Point cv_pt2 = ConvertCvPointToPixel(pt2);
    arrowedLine(paint_area_, cv_pt1, cv_pt2, color, thickness, line_type, 0, tip_length);
}

void CvCanvas::DrawRectangle(CPoint pt1, CPoint pt2, const cv::Scalar &color, int thickness, int line_type)
{
    cv::Point cv_pt1 = ConvertCvPointToPixel(pt1);
    cv::Point cv_pt2 = ConvertCvPointToPixel(pt2);
    rectangle(paint_area_, cv_pt1, cv_pt2, color, thickness, line_type);
}

void CvCanvas::DrawPolyline(const std::vector<CPoint> &points, bool is_closed, const cv::Scalar &color, int thickness, int line_type)
{
    assert(points.size() <= max_polygon_pt_num);

    cv::Point pts[1][max_polygon_pt_num];
    for (int i = 0; i < points.size(); ++i)
        pts[0][i] = ConvertCvPointToPixel(points[i]);

    const cv::Point *ppt[1] = {pts[0]};
    int npt[] = {static_cast<int>(points.size())};
    polylines(paint_area_, ppt, npt, 1, is_closed, color, thickness, line_type);
}

void CvCanvas::DrawEllipse(CPoint center, cv::Size axes, double angle, double start_angle, double end_angle, const cv::Scalar &color, int thickness, int line_type)
{
    cv::Point cv_center = ConvertCvPointToPixel(center);
    ellipse(paint_area_, cv_center, axes, angle, start_angle, end_angle, color, thickness, line_type);
}

void CvCanvas::FillConvexPoly(const std::vector<CPoint> &points, const cv::Scalar &color, int line_type)
{
    assert(points.size() <= max_polygon_pt_num);

    cv::Point pts[max_polygon_pt_num];
    for (int i = 0; i < points.size(); ++i)
        pts[i] = ConvertCvPointToPixel(points[i]);

    fillConvexPoly(paint_area_, pts, points.size(), color, line_type);
}

void CvCanvas::FillPoly(const std::vector<CPoint> &points, const cv::Scalar &color, int line_type)
{
    assert(points.size() <= max_polygon_pt_num);

    cv::Point pts[1][max_polygon_pt_num];
    for (int i = 0; i < points.size(); ++i)
        pts[0][i] = ConvertCvPointToPixel(points[i]);

    const cv::Point *ppt[1] = {pts[0]};
    int npt[] = {static_cast<int>(points.size())};

    fillPoly(paint_area_, ppt, npt, 1, color, line_type);
}

void CvCanvas::WriteText(const std::string &text, CPoint pos, double font_scale, const cv::Scalar &color, int thickness, int line_type)
{
    cv::Point text_pos = ConvertCvPointToPixel(pos);
    putText(paint_area_, text, text_pos, FONT_HERSHEY_SIMPLEX, font_scale, color, thickness, line_type);
}
} // namespace librav
