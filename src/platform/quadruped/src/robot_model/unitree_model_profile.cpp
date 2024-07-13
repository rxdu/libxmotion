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

  // values extracted from Unitree mujoco model: go2.xml
  profile.leg_hip_link = 0.0955;   // l_{abad}
  profile.leg_thigh_link = 0.213;  // l_{hip}
  profile.leg_calf_link = 0.213;   // l_{knee}

  profile.body_to_leg_offsets[LegIndex::kFrontRight] =
      Position3d(0.1934, -0.0465, 0.0);
  profile.body_to_leg_offsets[LegIndex::kFrontLeft] =
      Position3d(0.1934, 0.0465, 0.0);
  profile.body_to_leg_offsets[LegIndex::kRearRight] =
      Position3d(-0.1934, -0.0465, 0.0);
  profile.body_to_leg_offsets[LegIndex::kRearLeft] =
      Position3d(-0.1934, 0.0465, 0.0);

  return profile;
}
}  // namespace xmotion