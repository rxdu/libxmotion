/*
 * utest_polyline.cpp
 *
 * Created on: Nov 19, 2020 21:00
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "gtest/gtest.h"

#include "decomp/details/rect_grid_base.hpp"

using namespace robosw;

TEST(RectGridBaseTest, GridDouble) {
  const int size_x = 6;
  const int size_y = 8;

  RectGridBase<double> grid(size_x, size_y);

  ASSERT_TRUE(grid.SizeX() == size_x);
  ASSERT_TRUE(grid.SizeY() == size_y);

  grid.PrintGrid();

  grid.ResizeGrid(size_x * 2, size_y * 2);

  ASSERT_TRUE(grid.SizeX() == size_x * 2);
  ASSERT_TRUE(grid.SizeY() == size_y * 2);
}