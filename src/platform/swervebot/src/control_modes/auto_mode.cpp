/*
 * @file auto_mode.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/control_modes/auto_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
AutoMode::AutoMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to AutoMode");
}

void AutoMode::Update(ControlContext& context) {
  // clear js event queue
  AxisEvent axis_event;
  while (context.js_axis_queue->TryPop(axis_event)) {
  }

  // handle commands from higher level controller
}
}  // namespace xmotion