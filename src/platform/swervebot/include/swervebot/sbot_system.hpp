/*
 * @file sbot_system.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SBOT_SYSTEM_HPP
#define XMOTION_SBOT_SYSTEM_HPP

#include <atomic>
#include <memory>
#include <chrono>

#include "input_hid/joystick_handler.hpp"
#include "input_hid/hid_event_listener.hpp"

#include "swervebot/sbot_fsm.hpp"

namespace xmotion {
class SbotSystem {
  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;

 public:
  SbotSystem(const SbotConfig& config);
  ~SbotSystem() = default;

  // Run the application
  bool Initialize();
  void Run();
  void Stop();

 private:
  void ControlLoop();
  void OnJsButtonEvent(const JsButton& btn, const JxButtonEvent& event);
  void OnJsAxisEvent(const JsAxis& axis, const float& value);

  SbotConfig config_;
  std::atomic<bool> keep_running_{false};

  std::unique_ptr<JoystickHandler> joystick_;
  std::shared_ptr<HidEventListener> hid_event_listener_;

  std::shared_ptr<WsSbotBase> sbot_;

  std::thread control_thread_;
  std::atomic<bool> keep_control_loop_{true};
  std::unique_ptr<SbotFsm> fsm_;
};
}  // namespace xmotion

#endif  // XMOTION_SBOT_SYSTEM_HPP