#include <iostream>

#include "geometry/parametric_curve.hpp"

using namespace robosw;

int main() {
  Polyline polyline;
  polyline.AddPoint(0, 0);
  polyline.AddPoint(0.5, 0.25);
  polyline.AddPoint(1.0, 1.0);
  polyline.AddPoint(1.5, 1.75);
  polyline.AddPoint(2.0, 2);

  auto curve = CurveFitting::FitApproximateLengthCurve(polyline);

  for (double i = 0; i < 2.0; i += 0.5)
    std::cout << " s = " << i << " , " << curve.Evaluate(i) << std::endl;

  CvCanvas canvas(100);
  canvas.Resize(-1, 3, -1, 3);
  canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

  DrawPolyline(canvas, polyline);
  DrawParametricCurve(canvas, curve, 0.1, CvColors::red_color);

  canvas.Show();

  return 0;
}