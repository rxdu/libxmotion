#include <iostream>

#include "cvdraw/cvdraw.hpp"

using namespace xmotion;

int main() {
  CvCanvas canvas(10);
  canvas.Resize(-60, 60, -20, 20);
  canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

  canvas.DrawXYAxis();
  canvas.DrawReferenceGrid();

  canvas.DrawCurveFunction(
      {-50, .1, 50}, [](double x) -> double { return cos(x * .628f + 4) * 5; },
      CvColors::blue_color);

  std::vector<std::pair<double, double>> data_points;
  double x = -50;
  while (x < 50) {
    data_points.emplace_back(x, 10 * sin(x * .628f + 4));
    x += 0.1;
  }
  canvas.DrawDataPoints(data_points, CvColors::red_color);

  // example from
  // http://sites.millersville.edu/bikenaga/calculus/parametric-equations/parametric-equations.html
  canvas.DrawParametricCurve(
      {-50, .1, 50}, [](double s) -> double { return s * s * s + s + 2; },
      [](double s) -> double { return 2 * s * s * s - 3 * s * s - 12 * s + 5; },
      CvColors::lime_color, 2);

  canvas.DrawCircularArc({0, 0}, 15, 45, 315, CvColors::maroon_color);

  CvIO::ShowImage(canvas.GetPaintArea());
  // canvas.Show();
  canvas.SavePaint("function_plot_demo");

  return 0;
}