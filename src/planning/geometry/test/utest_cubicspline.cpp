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

using namespace xmotion;

struct CubicSplineTest : testing::Test {
  CubicSplineTest() {
    knots.emplace_back(1, 2);
    knots.emplace_back(2, 3);
    knots.emplace_back(3, 5);
  }

  CubicSpline spline;
  std::vector<CubicSpline::Knot> knots;

  // Natual boundary spline
  // S(x) = 2 + 3/4 * (x - 1) + 1/4 * (x - 1)^3, x in [1,2]
  //      = 3 + 3/2 * (x - 2) + 3/4 * (x - 2)^2 - 1/4 * (x - 2)^3, x in [2,3]
  double CalculateNatual(double x, uint32_t derivative = 0) {
    if (x >= 1 && x <= 2) {
      double e = x - 1;
      if (derivative == 0)
        return 2 + 3.0 / 4.0 * e + 1.0 / 4.0 * e * e * e;
      else if (derivative == 1)
        return 3.0 / 4.0 + 3.0 / 4.0 * e * e;
      else if (derivative == 2)
        return 3.0 / 2.0 * e;
      else if (derivative == 3)
        return 3.0 / 2.0;
    } else if (x >= 2 && x <= 3) {
      double e = x - 2;
      if (derivative == 0)
        return 3 + 3.0 / 2.0 * e + 3.0 / 4.0 * e * e - 1.0 / 4.0 * e * e * e;
      else if (derivative == 1)
        return 3.0 / 2.0 + 6.0 / 4.0 * e - 3.0 / 4.0 * e * e;
      else if (derivative == 2)
        return 6.0 / 4.0 - 6.0 / 4.0 * e;
      else if (derivative == 3)
        return -6.0 / 4.0;
    }
    return std::numeric_limits<double>::signaling_NaN();
  }

  // Clamped boundary spline
  // S(x) = 2 + 2 * (x - 1)  - 5/2 * (x - 1)^2 + 3/2 * (x - 1)^3, x in [1,2]
  //      = 3 + 3/2 * (x - 2) + 2 * (x - 2)^2 - 3/2 * (x - 2)^3, x in [2,3]
  double CalculateClamped(double x, uint32_t derivative = 0) {
    if (x >= 1 && x <= 2) {
      double e = x - 1;
      if (derivative == 0)
        return 2 + 2.0 * e - 5.0 / 2.0 * e * e + 3.0 / 2.0 * e * e * e;
      else if (derivative == 1)
        return 2.0 - 5.0 * e + 9.0 / 2.0 * e * e;
      else if (derivative == 2)
        return -5.0 + 9.0 * e;
      else if (derivative == 3)
        return 9.0;
    } else if (x >= 2 && x <= 3) {
      double e = x - 2;
      if (derivative == 0)
        return 3 + 3.0 / 2.0 * e + 2.0 * e * e - 3.0 / 2.0 * e * e * e;
      else if (derivative == 1)
        return 3.0 / 2.0 + 4.0 * e - 9.0 / 2.0 * e * e;
      else if (derivative == 2)
        return 4.0 - 9.0 * e;
      else if (derivative == 3)
        return -9.0;
    }

    return std::numeric_limits<double>::signaling_NaN();
  }
};

TEST_F(CubicSplineTest, InterpolateNatual) {
  spline.Interpolate(knots);
  ASSERT_TRUE(spline.GetKnots().size() == 3);

  auto coeffs = spline.GetCoefficients();
  ASSERT_FLOAT_EQ(coeffs(0, 0), 2);
  ASSERT_FLOAT_EQ(coeffs(1, 0), 3);
  ASSERT_FLOAT_EQ(coeffs(0, 1), 0.75);
  ASSERT_FLOAT_EQ(coeffs(1, 1), 1.5);
  ASSERT_FLOAT_EQ(coeffs(0, 2), 0);
  ASSERT_FLOAT_EQ(coeffs(1, 2), 0.75);
  ASSERT_FLOAT_EQ(coeffs(0, 3), 0.25);
  ASSERT_FLOAT_EQ(coeffs(1, 3), -0.25);
}

TEST_F(CubicSplineTest, InterpolateClamped) {
  spline.Interpolate(2, 1, knots);
  ASSERT_TRUE(spline.GetKnots().size() == 3);

  auto coeffs = spline.GetCoefficients();
  ASSERT_FLOAT_EQ(coeffs(0, 0), 2);
  ASSERT_FLOAT_EQ(coeffs(1, 0), 3);
  ASSERT_FLOAT_EQ(coeffs(0, 1), 2);
  ASSERT_FLOAT_EQ(coeffs(1, 1), 1.5);
  ASSERT_FLOAT_EQ(coeffs(0, 2), -2.5);
  ASSERT_FLOAT_EQ(coeffs(1, 2), 2);
  ASSERT_FLOAT_EQ(coeffs(0, 3), 1.5);
  ASSERT_FLOAT_EQ(coeffs(1, 3), -1.5);
}

TEST_F(CubicSplineTest, NatualEvaluate) {
  spline.Interpolate(knots);

  ASSERT_FLOAT_EQ(spline.Evaluate(1), 2);
  ASSERT_FLOAT_EQ(spline.Evaluate(2), 3);
  ASSERT_FLOAT_EQ(spline.Evaluate(3), 5);

  //  ASSERT_TRUE(std::isnan(spline.Evaluate(0)));
  //  ASSERT_TRUE(std::isnan(spline.Evaluate(4)));

  ASSERT_FLOAT_EQ(spline.Evaluate(1.5), CalculateNatual(1.5));
  ASSERT_FLOAT_EQ(spline.Evaluate(2.5), CalculateNatual(2.5));

  ASSERT_FLOAT_EQ(spline.Evaluate(1.5, 1), CalculateNatual(1.5, 1));
  ASSERT_FLOAT_EQ(spline.Evaluate(2.5, 1), CalculateNatual(2.5, 1));

  ASSERT_FLOAT_EQ(spline.Evaluate(1.5, 2), CalculateNatual(1.5, 2));
  ASSERT_FLOAT_EQ(spline.Evaluate(2.5, 2), CalculateNatual(2.5, 2));

  ASSERT_FLOAT_EQ(spline.Evaluate(1.5, 3), CalculateNatual(1.5, 3));
  ASSERT_FLOAT_EQ(spline.Evaluate(2.5, 3), CalculateNatual(2.5, 3));
}

TEST_F(CubicSplineTest, ClampedEvaluate) {
  spline.Interpolate(2, 1, knots);

  ASSERT_FLOAT_EQ(spline.Evaluate(1), 2);
  ASSERT_FLOAT_EQ(spline.Evaluate(2), 3);
  ASSERT_FLOAT_EQ(spline.Evaluate(3), 5);

  //  ASSERT_TRUE(std::isnan(spline.Evaluate(0)));
  //  ASSERT_TRUE(std::isnan(spline.Evaluate(4)));

  ASSERT_FLOAT_EQ(spline.Evaluate(1.5), CalculateClamped(1.5));
  ASSERT_FLOAT_EQ(spline.Evaluate(2.5), CalculateClamped(2.5));

  ASSERT_FLOAT_EQ(spline.Evaluate(1.5, 1), CalculateClamped(1.5, 1));
  ASSERT_FLOAT_EQ(spline.Evaluate(2.5, 1), CalculateClamped(2.5, 1));

  ASSERT_FLOAT_EQ(spline.Evaluate(1.5, 2), CalculateClamped(1.5, 2));
  ASSERT_FLOAT_EQ(spline.Evaluate(2.5, 2), CalculateClamped(2.5, 2));

  ASSERT_FLOAT_EQ(spline.Evaluate(1.5, 3), CalculateClamped(1.5, 3));
  ASSERT_FLOAT_EQ(spline.Evaluate(2.5, 3), CalculateClamped(2.5, 3));
}