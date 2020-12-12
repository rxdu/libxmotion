#include <iostream>
#include <vector>
#include <cmath>

#include "geometry/cspline.hpp"

#include "geometry/curve_viz.hpp"
#include "geometry/polygon.hpp"

using namespace robotnav;

int main()
{
    std::vector<CSpline::Knot> knots;

    for (int i = 0; i < 10; i++)
        knots.emplace_back(i + 0.5 * std::sin(i), i + std::cos(i * i));

    CSpline spline(knots);

    LightViz::ShowCubicSpline(spline, 0.01, 10);

    return 0;
}