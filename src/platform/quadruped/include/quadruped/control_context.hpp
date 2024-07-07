/*
 * @file control_context.hpp
 * @date 7/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_CONTROL_CONTEXT_HPP
#define QUADRUPED_MOTION_CONTROL_CONTEXT_HPP

#include <memory>

#include "quadruped/robot_model/quadruped_model.hpp"

namespace xmotion {
struct ControlContext {
  std::shared_ptr<QuadrupedModel> robot_model;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_CONTROL_CONTEXT_HPP
