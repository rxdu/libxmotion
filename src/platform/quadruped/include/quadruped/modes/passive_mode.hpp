/*
 * @file passive_mode.hpp
 * @date 7/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_PASSIVE_MODE_HPP
#define QUADRUPED_MOTION_PASSIVE_MODE_HPP

#include "quadruped/control_context.hpp"

namespace xmotion {
class PassiveMode {
 public:
  PassiveMode() = default;
  ~PassiveMode() = default;

  void Update(ControlContext &context);
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_PASSIVE_MODE_HPP
