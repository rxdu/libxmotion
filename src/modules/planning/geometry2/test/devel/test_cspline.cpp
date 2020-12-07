#include <iostream>
#include <vector>
#include <cmath>

#include "geometry/cspline.hpp"

using namespace robotnav;

int main()
{
    std::vector<CSpline::Knot> knots;

    for (int i = 0; i < 10; i++)
        knots.emplace_back(i + 0.5 * std::sin(i), i + std::cos(i * i));

    // std::cout << "knot size: " << knots.size() << std::endl;

    CSpline spline(knots);

    for (double x = knots.front().x; x < knots.back().x; x += 0.01)
    {
        std::cout << x << " , " << spline.Evaluate(x) << std::endl;
    }

    CSpline spline2(spline);
    
    CSpline spline3;
    spline3 = spline;

    CSpline spline4(CSpline(knots));

    CSpline spline5;
    spline5 = CSpline(knots);

    return 0;
}