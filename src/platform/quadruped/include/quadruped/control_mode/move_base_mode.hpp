/*
 * @file move_base_mode.hpp
 * @date 7/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_MOVE_BASE_MODE_HPP
#define QUADRUPED_MOTION_MOVE_BASE_MODE_HPP

#include "quadruped/control_context.hpp"

namespace xmotion {
class MoveBaseMode {
 public:
  void Update(ControlContext &context);
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_MOVE_BASE_MODE_HPP
