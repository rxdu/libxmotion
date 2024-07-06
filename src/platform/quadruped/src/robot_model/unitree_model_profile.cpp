/*
 * unitree_model_profile.cpp
 *
 * Created on 7/6/24 10:53 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/robot_model/unitree_model_profile.hpp"

namespace xmotion {
UnitreeModelProfile UnitreeDogs::GetGo2Profile() {
  UnitreeModelProfile profile("Go2");

  profile.leg_l1 = 0.0;

  return profile;
}
}  // namespace xmotion