/*
 * cv_canvas.hpp
 *
 * Created on: Jan 04, 2019 10:40
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef CV_CANVAS_HPP
#define CV_CANVAS_HPP

#include <string>
#include <cstdint>
#include <vector>
#include <utility>
#include <functional>

#include <opencv2/opencv.hpp>

#include "cvdraw/cv_colors.hpp"

namespace xmotion {
struct CPoint {
  CPoint(double _x = 0, double _y = 0) : x(_x), y(_y) {}

  double x;
  double y;

  friend std::ostream &operator<<(std::ostream &os, const CPoint &pos) {
    os << "pixel (x, y): " << pos.x << " , " << pos.y;
    return os;
  }
};

struct CRange {
  CRange(double min = 0, double step = 0, double max = 0)
      : min_value(min), inc_value(step), max_value(max) {}

  double min_value = 0.0;
  double inc_value = 0.0;
  double max_value = 0.0;
};

class CvCanvas {
 public:
  enum class DrawMode {
    Raster,            // use pixel coordinate directly
    Vector,            // x,y mapped to [0,1]
    Geometry,          // geometric x,y in meters
    GeometryInvertedY  // geometric x,y in meters with y inverted
  };

 public:
  CvCanvas(cv::Mat background, std::string name = "CvCanvas");
  CvCanvas(int32_t px, int32_t py, cv::Scalar bg_color = CvColors::bg_color,
           std::string name = "CvCanvas");
  CvCanvas(int32_t ppu, cv::Scalar bg_color = CvColors::bg_color,
           std::string name = "CvCanvas");

  void Clear();
  void Resize(double px, double py);
  void Resize(double xmin, double xmax, double ymin, double ymax);
  void FillBackgroundColor(cv::Scalar bg_color);

  int32_t GetPPU() const { return ppu_; }

  int32_t GetCanvasSizeX() const { return canvas_size_x_; }

  int32_t GetCanvasSizeY() const { return canvas_size_y_; }

  void GetCanvasSpan(double &xspan, double &yspan);
  void GetCanvasRange(double &xmin, double &xmax, double &ymin, double &ymax);

  cv::Mat GetPaintArea() { return paint_area_; }

  cv::Mat GetROIofPaintArea(double cx, double cy, double xspan, double yspan);
  cv::Mat GetROIofPaintArea(double cx, double cy, double ratio);

  cv::Point ConvertGeometryPointToPixel(double xi, double yi);
  cv::Point ConvertNormalizedPointToPixel(double xi, double yi);

  void SavePaint(std::string filename);

  void Show(bool use_img_size = false);
  void ShowFrame(int32_t frame_period_ms = 0);

  void SetMode(DrawMode mode) { draw_mode_ = mode; }

  // draw primitives
  void DrawPoint(CPoint center, int radius = 3,
                 const cv::Scalar &color = CvColors::default_pt_color,
                 int line_type = cv::LINE_AA);
  void DrawCircle(CPoint center, int radius,
                  const cv::Scalar &color = CvColors::default_pt_color,
                  int thickness = 1, int line_type = cv::LINE_AA);
  void DrawCircularArc(CPoint center, double radius, double start_angle,
                       double end_angle,
                       const cv::Scalar &color = CvColors::default_ln_color,
                       int thickness = 1, int line_type = cv::LINE_AA);
  void DrawLine(CPoint pt1, CPoint pt2,
                const cv::Scalar &color = CvColors::default_ln_color,
                int thickness = 1, int line_type = cv::LINE_AA);
  void DrawArrowedLine(CPoint pt1, CPoint pt2, double tip_length = 0.1,
                       const cv::Scalar &color = CvColors::default_ln_color,
                       int thickness = 1, int line_type = cv::LINE_AA);

  void DrawRectangle(CPoint pt1, CPoint pt2,
                     const cv::Scalar &color = CvColors::default_ln_color,
                     int thickness = 1, int line_type = cv::LINE_AA);
  void DrawPolyline(const std::vector<CPoint> &points, bool isClosed,
                    const cv::Scalar &color = CvColors::default_ln_color,
                    int thickness = 1, int line_type = cv::LINE_AA);
  void DrawEllipse(CPoint center, cv::Size axes, double angle,
                   double start_angle, double end_angle,
                   const cv::Scalar &color = CvColors::default_ln_color,
                   int thickness = 1, int line_type = cv::LINE_AA);

  void FillConvexPoly(const std::vector<CPoint> &points,
                      const cv::Scalar &color = CvColors::default_ln_color,
                      int line_type = cv::LINE_AA);
  void FillPoly(const std::vector<CPoint> &points,
                const cv::Scalar &color = CvColors::default_ln_color,
                int line_type = cv::LINE_AA);

  void WriteText(const std::string &text, CPoint pos, double font_scale = 1,
                 const cv::Scalar &color = CvColors::black_color,
                 int thickness = 1, int line_type = cv::LINE_AA);

  // function plotting
  void DrawXYAxis(double arrow_size = 0.01);
  void DrawReferenceGrid(double spacing_unit = 10,
                         const cv::Scalar &color = CvColors::gray_color);
  void DrawDataPoints(const std::vector<std::pair<double, double>> &data_points,
                      const cv::Scalar &color = CvColors::default_ln_color,
                      int thickness = 1, int line_type = cv::LINE_AA);
  void DrawCurveFunction(CRange range, std::function<double(double)> func,
                         const cv::Scalar &color = CvColors::default_ln_color,
                         int thickness = 1, int line_type = cv::LINE_AA);
  void DrawParametricCurve(CRange range, std::function<double(double)> funcx,
                           std::function<double(double)> funcy,
                           const cv::Scalar &color = CvColors::default_ln_color,
                           int thickness = 1, int line_type = cv::LINE_AA);

 private:
  int32_t ppu_ = 10;

  // size parameters
  double xmin_ = 0.0;
  double xmax_ = 80.0;
  double ymin_ = 0.0;
  double ymax_ = 60.0;
  double xspan_ = 80.0;
  double yspan_ = 60.0;
  int32_t canvas_size_x_ = 800;
  int32_t canvas_size_y_ = 600;

  cv::Mat paint_area_;
  cv::Mat init_paint_;
  cv::Scalar bg_color_;
  std::string canvas_name_;

  DrawMode draw_mode_ = DrawMode::Raster;

  bool img_bg_mode_ = false;
  static constexpr int max_polygon_pt_num = 100;

  void InitCanvas();

  inline void ClampToCanvasSize(int32_t &x, int32_t y) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= canvas_size_x_) x = canvas_size_x_ - 1;
    if (y >= canvas_size_y_) y = canvas_size_y_ - 1;
  }

  inline cv::Point ConvertCvPointToPixel(CPoint pt) {
    int32_t x, y;

    if (draw_mode_ == DrawMode::Raster) {
      x = static_cast<int32_t>(pt.x);
      y = static_cast<int32_t>(pt.y);
    } else if (draw_mode_ == DrawMode::Geometry) {
      x = (pt.x - xmin_) / xspan_ * canvas_size_x_;
      y = (pt.y - ymin_) / yspan_ * canvas_size_y_;
    } else if (draw_mode_ == DrawMode::GeometryInvertedY) {
      x = (pt.x - xmin_) / xspan_ * canvas_size_x_;
      y = canvas_size_y_ - (pt.y - ymin_) / yspan_ * canvas_size_y_;
    } else  // if (draw_mode_ == DrawMode::Vector)
    {
      x = pt.x * canvas_size_x_;
      y = pt.y * canvas_size_y_;
    }

    ClampToCanvasSize(x, y);
    return cv::Point(x, y);
  }
};
}  // namespace xmotion

#endif /* CV_CANVAS_HPP */
