#include <iostream>
#include <vector>
#include <cmath>

#include "geometry/cspline.hpp"
#include "geometry/geometry_draw.hpp"

using namespace autodrive;

int main()
{
    std::vector<CSpline::Knot> knots;

    for (int i = 0; i < 10; i++)
        knots.emplace_back(i + 0.5 * std::sin(i), i + std::cos(i * i));

    CSpline spline(knots);

    CvCanvas canvas(100);
    canvas.Resize(-1, 10, -10, 10);
    canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

    GeometryViz::DrawCubicSpline(canvas, spline);

    canvas.Show();

    return 0;
}