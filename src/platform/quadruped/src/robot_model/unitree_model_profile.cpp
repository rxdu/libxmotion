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
  UnitreeModelProfile profile;

  profile.name = "unitree_go2";

  profile.leg_hip_link = 0.0955;    // l_{abad}
  profile.leg_thigh_link = 0.213;  // l_{hip}
  profile.leg_calf_link = 0.213;    // l_{knee}

  return profile;
}
}  // namespace xmotion