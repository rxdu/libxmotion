/*
 * @file control_context.hpp
 * @date 7/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_CONTROL_CONTEXT_HPP
#define QUADRUPED_MOTION_CONTROL_CONTEXT_HPP

#include "quadruped/robot_model/unitree_dog.hpp"

namespace xmotion {
class ControlContext {
 public:
  double linear_velocity;
  double angular_velocity;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_CONTROL_CONTEXT_HPP
