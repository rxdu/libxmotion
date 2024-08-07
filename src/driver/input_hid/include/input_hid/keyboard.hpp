/*
 * @file keyboard.hpp
 * @date 7/5/24
 * @brief A keyboard class that monitors keyboard events using polling way
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_KEYBOARD_HPP
#define XMOTION_KEYBOARD_HPP

#include <string>
#include <thread>
#include <atomic>

#include "interface/driver/keyboard_interface.hpp"
#include "input_hid/keyboard_mapping.hpp"

namespace xmotion {
class Keyboard : public KeyboardInterface {
 public:
  Keyboard() = default;
  ~Keyboard();

  // do not allow copy
  Keyboard(const Keyboard&) = delete;
  Keyboard& operator=(const Keyboard&) = delete;

  // public methods
  bool StartMonitoring(const std::string& event_name) override;
  void RegisterKeyEventCallback(KeyEventCallback callback) override;

  void PollEvent() override;

 private:
  int fd_;
  std::thread io_thread_;
  std::atomic<bool> keep_running_{false};

  KeyEventCallback key_event_callback_;
};
}  // namespace xmotion

#endif  // XMOTION_KEYBOARD_HPP
