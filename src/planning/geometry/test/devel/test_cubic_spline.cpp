#include <iostream>
#include <vector>
#include <cmath>

#include "geometry/cubic_spline.hpp"

using namespace xmotion;

int main() {
  std::vector<CubicSpline::Knot> knots;

  knots.emplace_back(1, 2);
  knots.emplace_back(2, 3);
  knots.emplace_back(3, 5);

  std::cout << "knot size: " << knots.size() << std::endl;

  CubicSpline natual_spline(knots);

  std::cout << "---------------" << std::endl;

  CubicSpline clamped_spline;
  clamped_spline.Interpolate(2, 1, knots);

  CvCanvas canvas(100);
  canvas.Resize(-1, 10, -10, 10);
  canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);
  DrawCubicSpline(canvas, natual_spline);
  DrawCubicSpline(canvas, clamped_spline, 0.01, CvColors::cyan_color);
  canvas.Show();

  return 0;
}