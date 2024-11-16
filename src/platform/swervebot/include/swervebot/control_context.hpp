/*
 * @file control_context.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_CONTROL_CONTEXT_HPP
#define XMOTION_CONTROL_CONTEXT_HPP

#include <memory>

#include "interface/driver/joystick_interface.hpp"

#include "swervebot/sbot_config.hpp"
#include "swervebot/ws_sbot_base.hpp"

#include "event/thread_safe_queue.hpp"

namespace xmotion {
struct AxisEvent {
  JsAxis axis;
  float value;
};

struct ControlContext {
  SbotConfig config;
  std::shared_ptr<WsSbotBase> robot_base;
  // for mode switch (consumed in main thread)
  std::shared_ptr<ThreadSafeQueue<JsButton>> fsm_js_button_press_queue;
  // for motion control (consumed in control thread)
  std::shared_ptr<ThreadSafeQueue<AxisEvent>> fsm_js_axis_move_queue;
};
}  // namespace xmotion

#endif  // XMOTION_CONTROL_CONTEXT_HPP