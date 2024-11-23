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
#include "robot_base/kinematics/swerve_drive_kinematics.hpp"
#include "input_sbus/sbus_receiver.hpp"

#include "event/thread_safe_queue.hpp"

namespace xmotion {
struct AxisEvent {
  JsAxis axis;
  float value;
};

struct UserCommand {
  float vx;
  float vy;
  float wz;
};

struct RobotFeedback {
  float x;
  float y;
  float theta;
};

struct ControlContext {
  SbotConfig config;
  std::shared_ptr<WsSbotBase> robot_base;
  // event thread -> main thread
  std::shared_ptr<ThreadSafeQueue<JsButton>> js_button_queue;
  std::shared_ptr<ThreadSafeQueue<AxisEvent>> js_axis_queue;
  std::shared_ptr<ThreadSafeQueue<SbusMessage>> sbus_rc_queue;
  // main thread -> control thread
  std::shared_ptr<ThreadSafeQueue<UserCommand>> command_queue;
  // control thread -> main thread
  std::shared_ptr<ThreadSafeQueue<RobotFeedback>> feedback_queue;
};
}  // namespace xmotion

#endif  // XMOTION_CONTROL_CONTEXT_HPP