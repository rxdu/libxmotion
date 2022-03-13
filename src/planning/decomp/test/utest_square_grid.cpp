/*
 * utest_polyline.cpp
 *
 * Created on: Nov 19, 2020 21:00
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "gtest/gtest.h"

#include "decomp/square_grid.hpp"

using namespace robosw;

TEST(SquareGridTest, BasicConstruct) {
  SquareGrid sgrid(6, 8, 0.5);

  ASSERT_TRUE(sgrid.SizeX() == 6);
  ASSERT_TRUE(sgrid.SizeY() == 8);
  ASSERT_FLOAT_EQ(sgrid.GetCellSize(), 0.5);
}

TEST(SquareGridTest, MatrixConstruct) {
  // row: 10, col: 8
  Eigen::MatrixXd mat = Eigen::MatrixXd::Random(10, 8);
  SquareGrid sgrid(mat, 2, 0.2);

  ASSERT_TRUE(sgrid.SizeX() == 4);
  ASSERT_TRUE(sgrid.SizeY() == 5);
  ASSERT_FLOAT_EQ(sgrid.GetCellSize(), 0.2);
}