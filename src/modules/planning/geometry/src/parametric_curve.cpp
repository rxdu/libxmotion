/*
 * parametric_curve.cpp
 *
 * Created on: Oct 20, 2018 08:44
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "geometry/parametric_curve.hpp"

#include <cassert>

namespace robotnav {
ParametricCurve::ParametricCurve(Polyline center_polyline)
    : polyline_(center_polyline) {}

ParametricCurve::ParametricCurve(CubicSpline xspline, CubicSpline yspline,
                                 double sf)
    : x_spline_(xspline), y_spline_(yspline), total_length_(sf) {}

SimplePoint2 ParametricCurve::Evaluate(double s, int32_t derivative) const {
  return SimplePoint2(x_spline_.Evaluate(s, derivative),
                      y_spline_.Evaluate(s, derivative));
}

void ParametricCurve::GetPositionVector(double s, double &x, double &y) const {
  auto state = Evaluate(s);
  x = state.x();
  y = state.y();
}

void ParametricCurve::GetTangentVector(double s, double &x, double &y) const {
  double vx = x_spline_.Evaluate(s, 1);
  double vy = y_spline_.Evaluate(s, 1);
  double norm = std::hypot(vx, vy);
  x = vx / norm;
  y = vy / norm;
}

/////////////////////////////////////////////////////////////////////////////////

ParametricCurve CurveFitting::FitApproximateLengthCurve(Polyline polyline) {
  std::vector<double> distances;
  distances.push_back(0.0);

  double accumulated = 0.0;
  for (int32_t i = 0; i < polyline.GetPointNumer() - 1; ++i) {
    double dist =
        std::hypot(polyline.GetPoint(i).x() - polyline.GetPoint(i + 1).x(),
                   polyline.GetPoint(i).y() - polyline.GetPoint(i + 1).y());
    accumulated += dist;
    distances.push_back(accumulated);
  }

  std::vector<CubicSpline::Knot> xknots, yknots;
  for (int32_t i = 0; i < polyline.GetPointNumer(); ++i) {
    xknots.emplace_back(distances[i], polyline.GetPoint(i).x());
    yknots.emplace_back(distances[i], polyline.GetPoint(i).y());
  }

  return ParametricCurve(CubicSpline(xknots), CubicSpline(yknots), accumulated);
}

ParametricCurve CurveFitting::FitTimedCurve(std::vector<double> x,
                                            std::vector<double> y,
                                            std::vector<double> t) {
  assert((t.size() > 2) && (x.size() == t.size()) && (y.size() == t.size()));

  std::vector<CubicSpline::Knot> xknots, yknots;
  for (int32_t i = 0; i < t.size(); ++i) {
    xknots.emplace_back(t[i], x[i]);
    yknots.emplace_back(t[i], y[i]);
  }

  return ParametricCurve(CubicSpline(xknots), CubicSpline(yknots), t.back());
}

#ifdef ENABLE_VISUAL
void DrawParametricCurve(CvCanvas &canvas, const ParametricCurve &pcurve,
                         double step, cv::Scalar ln_color, int32_t thickness) {
  std::vector<cv::Point2d> pts;

  for (double s = 0; s < pcurve.GetLength(); s += step)
    pts.emplace_back(pcurve.GetXSpline().Evaluate(s),
                     pcurve.GetYSpline().Evaluate(s));

  // std::cout << "intermediate points: " << pts.size() << std::endl;

  for (std::size_t i = 0; i < pts.size() - 1; ++i)
    canvas.DrawLine({pts[i].x, pts[i].y}, {pts[i + 1].x, pts[i + 1].y},
                    ln_color, thickness);
}
#endif
}  // namespace robotnav