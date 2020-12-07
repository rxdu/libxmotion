/*
 * utest_polyline.cpp
 *
 * Created on: Nov 19, 2020 21:00
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "gtest/gtest.h"

#include "geometry/cubic_spline.hpp"

using namespace robotnav;

struct CubicSplineTest : testing::Test {
  CubicSplineTest() {
    for (int i = 0; i < 10; i++)
      knots.emplace_back(i + 0.5 * std::sin(i), i + std::cos(i * i));
  }

  CubicSpline spline;
  std::vector<CubicSpline::Knot> knots;
};

TEST_F(CubicSplineTest, Interpolate) {
  spline.Interpolate(knots);
  ASSERT_TRUE(spline.GetKnots().size() == 10)
      << "Incorrect number of points in cubic spline";
}