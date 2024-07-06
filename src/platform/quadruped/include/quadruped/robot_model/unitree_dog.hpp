/*
 * @file unitree_dog.hpp
 * @date 7/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_UNITREE_DOG_HPP
#define QUADRUPED_MOTION_UNITREE_DOG_HPP

#include <eigen3/Eigen/Core>

#include <array>

#include "quadruped/robot_model/quadruped_model.hpp"
#include "quadruped/robot_model/unitree_motor.hpp"
#include "quadruped/robot_model/unitree_model_profile.hpp"

namespace xmotion {
class UnitreeDog : public QuadrupedModel {
 public:
  struct State {};

  UnitreeDog(const UnitreeModelProfile& profile);

 private:
  UnitreeModelProfile profile_;
  std::array<UnitreeMotor, 12> motors_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UNITREE_DOG_HPP
