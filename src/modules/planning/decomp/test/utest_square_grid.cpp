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

using namespace robotnav;

TEST(SquareGridTest, BasicConstruct) {
  SquareGrid sgrid(6, 8, 0.5);

  ASSERT_TRUE(sgrid.SizeX() == 6);
  ASSERT_TRUE(sgrid.SizeY() == 8);
  ASSERT_FLOAT_EQ(sgrid.GetCellSize(), 0.5);
}

TEST(SquareGridTest, MatrixConstruct) {
  Eigen::MatrixXd mat = Eigen::MatrixXd::Random(10, 8);
  SquareGrid sgrid(mat, 2);
  std::cout << "x: " << sgrid.SizeX() << " , y: " << sgrid.SizeY() << std::endl;
  ASSERT_TRUE(sgrid.SizeX() == 5);
  ASSERT_TRUE(sgrid.SizeY() == 4);
}