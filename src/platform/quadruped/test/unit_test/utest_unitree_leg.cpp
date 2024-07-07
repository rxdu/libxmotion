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
  UnitreeLeg leg(go2_profile_, LegIndex::kFrontLeft);
}