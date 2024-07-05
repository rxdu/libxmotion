/*
 * @file keyboard.hpp
 * @date 7/5/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_KEYBOARD_HPP
#define QUADRUPED_MOTION_KEYBOARD_HPP

#include <string>
#include <thread>
#include <atomic>

#include "interface/driver/keyboard_interface.hpp"

namespace xmotion {
class Keyboard : public KeyboardInterface {
 public:
  Keyboard(bool with_daemon = true);
  ~Keyboard();

  // do not allow copy
  Keyboard(const Keyboard&) = delete;
  Keyboard& operator=(const Keyboard&) = delete;

  // public methods
  bool StartMonitoring(const std::string& event_name) override;
  void RegisterKeyEventCallback(KeyEventCallback callback) override;

  void PollEvent() override;

  static std::string GetKeyName(KeyboardCode code);

 private:
  bool with_daemon_ = false;
  int fd_;
  std::thread io_thread_;
  std::atomic<bool> keep_running_{false};

  KeyEventCallback key_event_callback_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_KEYBOARD_HPP
