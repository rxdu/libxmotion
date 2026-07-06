/*
 * geometry_draw.cpp
 *
 * Copyright (c) 2018-2026 Ruixiang Du (rdu)
 */

#include "xmnav/viz/geometry_draw.hpp"

#include <cassert>

using namespace quickviz;

namespace xmotion {
void DrawPolyline(CvCanvas &canvas, const Polyline &polyline, bool show_dot,
                  cv::Scalar ln_color, int32_t thickness) {
  std::size_t pt_num = polyline.GetPointNumer();

  if (pt_num == 0) return;

  if (pt_num > 1) {
    for (std::size_t i = 0; i < pt_num - 1; ++i) {
      CPoint pt1(polyline.GetPoint(i).x(), polyline.GetPoint(i).y());
      CPoint pt2(polyline.GetPoint(i + 1).x(), polyline.GetPoint(i + 1).y());
      canvas.DrawLine(pt1, pt2, ln_color, thickness);
    }
  }

  if (show_dot) {
    for (std::size_t i = 0; i < pt_num; ++i)
      canvas.DrawPoint({polyline.GetPoint(i).x(), polyline.GetPoint(i).y()}, 1,
                       CvColors::red_color);
  }
}

void DrawCubicSpline(quickviz::CvCanvas &canvas, const CubicSpline &spline, double step,
                     cv::Scalar ln_color, int32_t thickness) {
  std::vector<cv::Point2d> pts;
  std::vector<CubicSpline::Knot> knots(spline.GetKnots());
  for (double x = knots.front().x(); x < knots.back().x(); x += step)
    pts.emplace_back(x, spline.Evaluate(x));

  std::cout << "intermediate points: " << pts.size() << std::endl;

  for (std::size_t i = 0; i < pts.size() - 1; ++i)
    canvas.DrawLine({pts[i].x, pts[i].y}, {pts[i + 1].x, pts[i + 1].y},
                    ln_color, thickness);
}

void DrawParametricCurve(quickviz::CvCanvas &canvas,
                         const ParametricCurve &pcurve, double step,
                         cv::Scalar ln_color, int32_t thickness) {
  std::vector<cv::Point2d> pts;

  for (double s = 0; s < pcurve.GetLength(); s += step)
    pts.emplace_back(pcurve.GetXSpline().Evaluate(s),
                     pcurve.GetYSpline().Evaluate(s));

  // std::cout << "intermediate points: " << pts.size() << std::endl;

  for (std::size_t i = 0; i < pts.size() - 1; ++i)
    canvas.DrawLine({pts[i].x, pts[i].y}, {pts[i + 1].x, pts[i + 1].y},
                    ln_color, thickness);
}

void DrawPolygon(CvCanvas &canvas, const Polygon &polygon, bool show_dot,
                 cv::Scalar ln_color, int32_t thickness) {
  std::size_t pt_num = polygon.GetPointNumer();

  if (pt_num < 3) return;

  for (std::size_t i = 0; i < pt_num - 1; ++i) {
    CPoint pt1(polygon.GetPoint(i).x(), polygon.GetPoint(i).y());
    CPoint pt2(polygon.GetPoint(i + 1).x(), polygon.GetPoint(i + 1).y());
    canvas.DrawLine(pt1, pt2, ln_color, thickness);
  }
  CPoint last_pt(polygon.GetPoint(pt_num - 1).x(),
                 polygon.GetPoint(pt_num - 1).y());
  CPoint first_pt(polygon.GetPoint(0).x(), polygon.GetPoint(0).y());
  canvas.DrawLine(last_pt, first_pt, ln_color, thickness);

  if (show_dot) {
    for (std::size_t i = 0; i < pt_num; ++i)
      canvas.DrawPoint({polygon.GetPoint(i).x(), polygon.GetPoint(i).y()}, 1,
                       CvColors::red_color);
  }
}

void FillPolygon(CvCanvas &canvas, const Polygon &polygon, bool /*show_dot*/,
                 cv::Scalar fill_color, cv::Scalar /*ln_color*/,
                 int32_t /*thickness*/) {
  std::size_t pt_num = polygon.GetPointNumer();

  if (pt_num < 3) return;

  std::vector<CPoint> pts;
  for (int i = 0; i < polygon.GetPointNumer(); ++i)
    pts.emplace_back(polygon.GetPoint(i).x(), polygon.GetPoint(i).y());

  canvas.FillPoly(pts, fill_color);
}
}  // namespace xmotion
