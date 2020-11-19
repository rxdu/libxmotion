/*
 * utest_polyline.cpp
 *
 * Created on: Nov 19, 2020 21:00
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "gtest/gtest.h"

#include "geometry/polyline.hpp"

using namespace rnav;

struct PolylineTest : testing::Test {
  PolylineTest() {
    polyline.AddPoint(0, 0);
    polyline.AddPoint(0.5, 0.25);
    polyline.AddPoint(1.0, 1.0);
    polyline.AddPoint(1.5, 1.75);
    polyline.AddPoint(2.0, 2);
  }

  Polyline polyline;
};

TEST_F(PolylineTest, Evaluate) {
  ASSERT_TRUE(polyline.GetPoints().size() == 5)
      << "Incorrect number of points in polyline";
}