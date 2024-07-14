/*
 * test_unitree_leg.cpp
 *
 * Created on 7/7/24 2:26 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include "quadruped/robot_model/unitree_leg.hpp"

using namespace xmotion;

struct UnitreeLegTest : public ::testing::Test {
  UnitreeLegTest() : go2_profile_(UnitreeDogs::GetGo2Profile()) {}

  UnitreeModelProfile go2_profile_;
};

TEST_F(UnitreeLegTest, ConstructorTest) {
  UnitreeLeg leg(go2_profile_, LegIndex::kFrontRight);

  {
    auto input = JointPosition3d(0.2, 0.2, -0.2);

    auto pos = leg.GetFootPosition(input);

    std::cout << "pos: " << pos[0] << " " << pos[1] << " " << pos[2]
              << std::endl;

    auto q = leg.GetJointPosition(pos);
    std::cout << "q: " << q[0] << " " << q[1] << " " << q[2] << std::endl;

    EXPECT_FLOAT_EQ(input[0], q[0]);
    EXPECT_FLOAT_EQ(input[1], q[1]);
    EXPECT_FLOAT_EQ(input[2], q[2]);
  }

  {
    auto input = JointPosition3d(0.2, -0.2, -0.25);

    auto pos = leg.GetFootPosition(input);

    std::cout << "pos: " << pos[0] << " " << pos[1] << " " << pos[2]
              << std::endl;

    auto q = leg.GetJointPosition(pos);
    std::cout << "q: " << q[0] << " " << q[1] << " " << q[2] << std::endl;

    EXPECT_FLOAT_EQ(input[0], q[0]);
    EXPECT_FLOAT_EQ(input[1], q[1]);
    EXPECT_FLOAT_EQ(input[2], q[2]);
  }
}