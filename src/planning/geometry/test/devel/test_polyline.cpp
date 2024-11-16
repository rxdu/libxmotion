#include <iostream>

#include "geometry/polyline.hpp"

using namespace xmotion;
using namespace quickviz;

int main() {
  Polyline polyline;
  polyline.AddPoint(0, 0);
  polyline.AddPoint(0.5, 0.25);
  polyline.AddPoint(1.0, 1.0);
  polyline.AddPoint(1.5, 1.75);
  polyline.AddPoint(2.0, 2);

  CvCanvas canvas(100);
  canvas.Resize(-1, 3, -1, 3);
  canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);
  DrawPolyline(canvas, polyline);
  canvas.Show();

  return 0;
}