/*
 * @file keyboard_handler.hpp
 * @date 7/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_KEYBOARD_HANDLER_HPP
#define XMOTION_KEYBOARD_HANDLER_HPP

#include <string>

#include "interface/driver/hid_handler_interface.hpp"
#include "interface/driver/keyboard_interface.hpp"
#include "input_hid/details/keyboard_mapping.hpp"

namespace xmotion {
class KeyboardHandler : public HidInputInterface {
 public:
  KeyboardHandler(const std::string &input_event);
  ~KeyboardHandler();

  // do not allow copy
  KeyboardHandler(const KeyboardHandler &) = delete;
  KeyboardHandler &operator=(const KeyboardHandler &) = delete;

  // public methods
  using KeyEventCallback = KeyboardInterface::KeyEventCallback;
  void RegisterKeyEventCallback(KeyEventCallback callback);

  bool Open() override;
  void Close() override;
  bool IsOpened() const override;

  int GetFd() const override { return fd_; }

  void OnInputEvent() override;

 private:
  std::string device_;
  int fd_;
  std::vector<KeyEventCallback> key_event_callbacks_;
};
}  // namespace xmotion

#endif  // XMOTION_KEYBOARD_HANDLER_HPP
