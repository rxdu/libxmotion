/*
 * unitree_model_profiles.hpp
 *
 * Created on 7/6/24 10:22 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_UNITREE_MODEL_PROFILE_HPP
#define QUADRUPED_MOTION_UNITREE_MODEL_PROFILE_HPP

#include <string>

#include "quadruped/robot_model/quadruped_model.hpp"

namespace xmotion {
struct UnitreeModelProfile {
  std::string name = "unitree_dog";

  double mass = 0.0;
  Eigen::Matrix3d moment_of_inertia;

  // leg parameters
  double leg_hip_link = 0.0;
  double leg_thigh_link = 0.0;
  double leg_calf_link = 0.0;

  // body to leg offsets
  std::unordered_map<LegIndex, Position3d> body_to_leg_offsets;
  Position3d center_of_gravity_offset;
};

struct UnitreeDogs {
  static UnitreeModelProfile GetGo2Profile();
  static UnitreeModelProfile GetGo2SimProfile();
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UNITREE_MODEL_PROFILE_HPP
