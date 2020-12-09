#include <iostream>
#include <vector>
#include <cmath>

#include "geometry/cubic_spline.hpp"

using namespace robotnav;

int main() {
  std::vector<CubicSpline::Knot> knots;

  knots.emplace_back(1, 2);
  knots.emplace_back(2, 3);
  knots.emplace_back(3, 5);

  std::cout << "knot size: " << knots.size() << std::endl;

  CubicSpline spline(knots);

  //   for (double x = knots.front().x(); x < knots.back().x(); x += 0.01) {
  //     std::cout << x << " , " << spline.Evaluate(x) << std::endl;
  //   }

  //   std::cout << "---------------" << std::endl;

  //   std::vector<CubicSpline::Knot> knots2;

  //   knots2.emplace_back(0, 1);
  //   knots2.emplace_back(1, std::exp(1));
  //   knots2.emplace_back(2, std::exp(2));
  //   knots2.emplace_back(3, std::exp(3));

  //   std::cout << "knot size: " << knots2.size() << std::endl;

  //   CubicSpline spline2;
  //   spline2.Interpolate(1, std::exp(3), knots2);

  return 0;
}