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

namespace xmotion {
class UnitreeModelProfile {
 public:
  UnitreeModelProfile(const std::string& name) : name_(name) {}

  float leg_l1 = 0.0;

 private:
  std::string name_;
};

struct UnitreeDogs {
  static UnitreeModelProfile GetGo2Profile();
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UNITREE_MODEL_PROFILE_HPP
