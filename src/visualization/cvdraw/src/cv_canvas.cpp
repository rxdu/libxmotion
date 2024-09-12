/*
 * cv_canvas.cpp
 *
 * Created on: Jan 04, 2019 11:09
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "cvdraw/cv_canvas.hpp"

#include <cmath>
#include <cassert>

namespace xmotion {
using namespace cv;

CvCanvas::CvCanvas(cv::Mat background, std::string name)
    : init_paint_(background), canvas_name_(name) {
  canvas_size_x_ = background.cols;
  canvas_size_y_ = background.rows;
  img_bg_mode_ = true;

  InitCanvas();
}

CvCanvas::CvCanvas(int32_t px, int32_t py, cv::Scalar bg_color,
                   std::string name)
    : canvas_size_x_(px),
      canvas_size_y_(py),
      bg_color_(bg_color),
      canvas_name_(name) {
  InitCanvas();
}

CvCanvas::CvCanvas(int32_t ppu, cv::Scalar bg_color, std::string name)
    : ppu_(ppu), bg_color_(bg_color), canvas_name_(name) {
  // init with default values of canvas_size_x_ and canvas_size_y_
  InitCanvas();

  // use geometry mode
  draw_mode_ = DrawMode::GeometryInvertedY;
}

void CvCanvas::InitCanvas() {
  assert(canvas_size_x_ > 0 && canvas_size_y_ > 0);

  if (!img_bg_mode_) {
    if (!init_paint_.empty()) init_paint_.release();
    cv::Mat canvas(canvas_size_y_, canvas_size_x_, CV_8UC3, bg_color_);
    init_paint_ = canvas;
  }

  paint_area_ = init_paint_.clone();
}

void CvCanvas::Resize(double px, double py) {
  assert(px > 0 && py > 0);

  canvas_size_x_ = px;
  canvas_size_y_ = py;

  InitCanvas();
}

void CvCanvas::Resize(double xmin, double xmax, double ymin, double ymax) {
  assert(xmax > xmin && ymax > ymin);

  xmin_ = xmin;
  xmax_ = xmax;
  ymin_ = ymin;
  ymax_ = ymax;

  xspan_ = xmax - xmin;
  yspan_ = ymax - ymin;

  // do not update canvas info if using background image
  if (img_bg_mode_) return;

  canvas_size_x_ = xspan_ * ppu_;
  canvas_size_y_ = yspan_ * ppu_;

  InitCanvas();
}

void CvCanvas::Clear() {
  paint_area_.release();
  paint_area_ = init_paint_.clone();
}

void CvCanvas::FillBackgroundColor(cv::Scalar bg_color) {
  bg_color_ = bg_color;

  InitCanvas();
}

void CvCanvas::GetCanvasSpan(double &xspan, double &yspan) {
  xspan = xspan_;
  yspan = yspan_;
}

void CvCanvas::GetCanvasRange(double &xmin, double &xmax, double &ymin,
                              double &ymax) {
  xmin = xmin_;
  ymin = ymin_;

  xmax = xmax_;
  ymax = ymax_;
}

cv::Mat CvCanvas::GetROIofPaintArea(double cx, double cy, double xspan,
                                    double yspan) {
  double rxmin = cx - xspan / 2.0;
  double rxmax = cx + xspan / 2.0;
  double rymin = cy - yspan / 2.0;
  double rymax = cy + yspan / 2.0;

  // std::cout << rxmin << " , " << rxmax << " ; " << rymin << " , " << rymax <<
  // std::endl;

  auto tl_pt = ConvertGeometryPointToPixel(rxmin, rymax);
  auto br_pt = ConvertGeometryPointToPixel(rxmax, rymin);

  // std::cout << "tl: " << tl_pt.x << " , " << tl_pt.y << std::endl;
  // std::cout << "br: " << br_pt.x << " , " << br_pt.y << std::endl;

  int width = br_pt.x - tl_pt.x;
  int height = br_pt.y - tl_pt.y;

  double aspct_ratio = canvas_size_y_ / canvas_size_x_;
  int height2 = static_cast<int>(width * aspct_ratio);

  // std::cout << "width: " << width << " , height: " << height << std::endl;

  cv::Mat roiImage = paint_area_(cv::Rect(tl_pt.x, tl_pt.y, width, height2));

  cv::Mat output;
  roiImage.copyTo(output);

  return output;
}

cv::Mat CvCanvas::GetROIofPaintArea(double cx, double cy, double ratio) {
  double xspan = (xmax_ - xmin_) * ratio;
  double yspan = (ymax_ - ymin_) * ratio;

  double rxmin = cx - xspan / 2.0;
  double rxmax = cx + xspan / 2.0;
  double rymin = cy - yspan / 2.0;
  double rymax = cy + yspan / 2.0;

  // std::cout << rxmin << " , " << rxmax << " ; " << rymin << " , " << rymax <<
  // std::endl;

  auto tl_pt = ConvertGeometryPointToPixel(rxmin, rymax);
  auto br_pt = ConvertGeometryPointToPixel(rxmax, rymin);

  // std::cout << "tl: " << tl_pt.x << " , " << tl_pt.y << std::endl;
  // std::cout << "br: " << br_pt.x << " , " << br_pt.y << std::endl;

  int width = br_pt.x - tl_pt.x;
  int height = br_pt.y - tl_pt.y;

  // std::cout << "width: " << width << " , height: " << height << std::endl;

  if (tl_pt.x + width > canvas_size_x_) tl_pt.x = canvas_size_x_ - width - 1;
  if (tl_pt.y + height > canvas_size_y_) tl_pt.y = canvas_size_y_ - height - 1;

  cv::Mat roiImage = paint_area_(cv::Rect(tl_pt.x, tl_pt.y, width, height));

  cv::Mat output;
  roiImage.copyTo(output);

  return output;
}

cv::Point CvCanvas::ConvertGeometryPointToPixel(double xi, double yi) {
  int32_t x = (xi - xmin_) / xspan_ * canvas_size_x_;
  int32_t y;
  if (draw_mode_ != DrawMode::GeometryInvertedY)
    y = (yi - ymin_) / yspan_ * canvas_size_y_;
  else
    y = canvas_size_y_ - (yi - ymin_) / yspan_ * canvas_size_y_;

  ClampToCanvasSize(x, y);

  return cv::Point(x, y);
}

cv::Point CvCanvas::ConvertNormalizedPointToPixel(double xi, double yi) {
  int32_t x = xi * canvas_size_x_;
  int32_t y = yi * canvas_size_y_;

  ClampToCanvasSize(x, y);

  return cv::Point(x, y);
}

void CvCanvas::SavePaint(std::string filename) {
  imwrite(filename + ".png", paint_area_);
}

void CvCanvas::Show(bool use_img_size) {
  int flags = WINDOW_NORMAL;
  if (use_img_size) flags = WINDOW_AUTOSIZE;
  namedWindow(canvas_name_, flags | WINDOW_KEEPRATIO | WINDOW_GUI_EXPANDED);
  imshow(canvas_name_, paint_area_);

  waitKey(0);
  destroyWindow(canvas_name_);
}

void CvCanvas::ShowFrame(int32_t frame_period_ms) {
  imshow(canvas_name_, paint_area_);
  waitKey(frame_period_ms);
}

void CvCanvas::DrawPoint(CPoint center, int radius, const cv::Scalar &color,
                         int line_type) {
  cv::Point cv_center = ConvertCvPointToPixel(center);
  circle(paint_area_, cv_center, radius, color, cv::FILLED, line_type);
}

void CvCanvas::DrawCircle(CPoint center, int radius, const cv::Scalar &color,
                          int thickness, int line_type) {
  cv::Point cv_center = ConvertCvPointToPixel(center);
  circle(paint_area_, cv_center, radius, color, thickness, line_type);
}

void CvCanvas::DrawCircularArc(CPoint center, double radius, double start_angle,
                               double end_angle, const cv::Scalar &color,
                               int thickness, int line_type) {
  DrawParametricCurve(
      {start_angle / 180.0 * M_PI, 0.1 / 180.0 * M_PI,
       end_angle / 180.0 * M_PI},
      [center, radius](double s) -> double {
        return (radius * std::cos(s) + center.x);
      },
      [center, radius](double s) -> double {
        return (radius * std::sin(s) + center.y);
      },
      color, thickness, line_type);
}

void CvCanvas::DrawLine(CPoint pt1, CPoint pt2, const cv::Scalar &color,
                        int thickness, int line_type) {
  cv::Point cv_pt1 = ConvertCvPointToPixel(pt1);
  cv::Point cv_pt2 = ConvertCvPointToPixel(pt2);
  line(paint_area_, cv_pt1, cv_pt2, color, thickness, line_type);
}

void CvCanvas::DrawArrowedLine(CPoint pt1, CPoint pt2, double tip_length,
                               const cv::Scalar &color, int thickness,
                               int line_type) {
  cv::Point cv_pt1 = ConvertCvPointToPixel(pt1);
  cv::Point cv_pt2 = ConvertCvPointToPixel(pt2);
  arrowedLine(paint_area_, cv_pt1, cv_pt2, color, thickness, line_type, 0,
              tip_length);
}

void CvCanvas::DrawRectangle(CPoint pt1, CPoint pt2, const cv::Scalar &color,
                             int thickness, int line_type) {
  cv::Point cv_pt1 = ConvertCvPointToPixel(pt1);
  cv::Point cv_pt2 = ConvertCvPointToPixel(pt2);
  rectangle(paint_area_, cv_pt1, cv_pt2, color, thickness, line_type);
}

void CvCanvas::DrawPolyline(const std::vector<CPoint> &points, bool is_closed,
                            const cv::Scalar &color, int thickness,
                            int line_type) {
  assert(points.size() <= max_polygon_pt_num);

  cv::Point pts[1][max_polygon_pt_num];
  for (int i = 0; i < points.size(); ++i)
    pts[0][i] = ConvertCvPointToPixel(points[i]);

  const cv::Point *ppt[1] = {pts[0]};
  int npt[] = {static_cast<int>(points.size())};
  polylines(paint_area_, ppt, npt, 1, is_closed, color, thickness, line_type);
}

void CvCanvas::DrawEllipse(CPoint center, cv::Size axes, double angle,
                           double start_angle, double end_angle,
                           const cv::Scalar &color, int thickness,
                           int line_type) {
  cv::Point cv_center = ConvertCvPointToPixel(center);
  ellipse(paint_area_, cv_center, axes, angle, start_angle, end_angle, color,
          thickness, line_type);
}

void CvCanvas::FillConvexPoly(const std::vector<CPoint> &points,
                              const cv::Scalar &color, int line_type) {
  assert(points.size() <= max_polygon_pt_num);

  cv::Point pts[max_polygon_pt_num];
  for (int i = 0; i < points.size(); ++i)
    pts[i] = ConvertCvPointToPixel(points[i]);

  fillConvexPoly(paint_area_, pts, points.size(), color, line_type);
}

void CvCanvas::FillPoly(const std::vector<CPoint> &points,
                        const cv::Scalar &color, int line_type) {
  assert(points.size() <= max_polygon_pt_num);

  cv::Point pts[1][max_polygon_pt_num];
  for (int i = 0; i < points.size(); ++i)
    pts[0][i] = ConvertCvPointToPixel(points[i]);

  const cv::Point *ppt[1] = {pts[0]};
  int npt[] = {static_cast<int>(points.size())};

  fillPoly(paint_area_, ppt, npt, 1, color, line_type);
}

void CvCanvas::WriteText(const std::string &text, CPoint pos, double font_scale,
                         const cv::Scalar &color, int thickness,
                         int line_type) {
  cv::Point text_pos = ConvertCvPointToPixel(pos);
  putText(paint_area_, text, text_pos, FONT_HERSHEY_SIMPLEX, font_scale, color,
          thickness, line_type);
}

void CvCanvas::DrawXYAxis(double arrow_size) {
  DrawArrowedLine({xmin_ + 1, 0}, {xmax_ - 1, 0}, arrow_size,
                  CvColors::red_color);
  DrawArrowedLine({0, ymin_ + 1}, {0, ymax_ - 1}, arrow_size * xspan_ / yspan_,
                  CvColors::green_color);
}

void CvCanvas::DrawReferenceGrid(double spacing_unit, const cv::Scalar &color) {
  double xspacing = spacing_unit;
  double yspacing = spacing_unit;

  // draw vertical lines
  double xpos = 0;
  while (xpos < xmax_) {
    if (xpos != 0) DrawLine({xpos, ymin_ + 1.0}, {xpos, ymax_ - 1.0}, color);
    xpos += xspacing;
  }
  xpos = 0;
  while (xpos > xmin_) {
    if (xpos != 0) DrawLine({xpos, ymin_ + 1.0}, {xpos, ymax_ - 1.0}, color);
    xpos -= xspacing;
  }

  // draw horizontal lines
  double ypos = 0;
  while (ypos < xmax_) {
    if (ypos != 0) DrawLine({xmin_ + 1, ypos}, {xmax_ - 1, ypos}, color);
    ypos += yspacing;
  }
  ypos = 0;
  while (ypos > xmin_) {
    if (ypos != 0) DrawLine({xmin_ + 1, ypos}, {xmax_ - 1, ypos}, color);
    ypos -= yspacing;
  }
}

void CvCanvas::DrawDataPoints(
    const std::vector<std::pair<double, double>> &data_points,
    const cv::Scalar &color, int thickness, int line_type) {
  for (std::size_t i = 0; i < data_points.size() - 1; ++i) {
    DrawLine({data_points[i].first, data_points[i].second},
             {data_points[i + 1].first, data_points[i + 1].second}, color,
             thickness, line_type);
  }
}

void CvCanvas::DrawCurveFunction(CRange range,
                                 std::function<double(double)> func,
                                 const cv::Scalar &color, int thickness,
                                 int line_type) {
  assert(range.min_value >= xmin_ && range.max_value <= xmax_);

  std::vector<std::pair<double, double>> data_points;
  double x = range.min_value;
  while (x < range.max_value) {
    data_points.emplace_back(x, func(x));
    x += range.inc_value;
  }

  DrawDataPoints(data_points, color, thickness, line_type);
}

void CvCanvas::DrawParametricCurve(CRange range,
                                   std::function<double(double)> funcx,
                                   std::function<double(double)> funcy,
                                   const cv::Scalar &color, int thickness,
                                   int line_type) {
  std::vector<std::pair<double, double>> data_points;
  double s = range.min_value;
  while (s < range.max_value) {
    data_points.emplace_back(funcx(s), funcy(s));
    s += range.inc_value;
  }
  DrawDataPoints(data_points, color, thickness, line_type);
}
}  // namespace xmotion
