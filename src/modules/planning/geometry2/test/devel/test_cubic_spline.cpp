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

  return 0;
}