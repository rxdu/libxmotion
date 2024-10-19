/*
 * @file joystick_handler.hpp
 * @date 10/19/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_JOYSTICK_HANDLER_HPP
#define XMOTION_JOYSTICK_HANDLER_HPP

#include <string>

#include "interface/driver/hid_handler_interface.hpp"
#include "interface/driver/joystick_interface.hpp"

namespace xmotion {
class JoystickHandler : public HidInputInterface {
 public:
  JoystickHandler(const std::string &input_event);
  ~JoystickHandler();

  // do not allow copy
  JoystickHandler(const JoystickHandler &) = delete;
  JoystickHandler &operator=(const JoystickHandler &) = delete;

  // public methods
  using JoystickButtonEventCallback = JoystickInterface::ButtonEventCallback;
  using JoystickAxisEventCallback = JoystickInterface::AxisEventCallback;

  void RegisterJoystickButtonEventCallback(
      JoystickButtonEventCallback callback);
  void RegisterJoystickAxisEventCallback(JoystickAxisEventCallback callback);

  bool Open() override;
  void Close() override;
  bool IsOpened() const override;

  int GetFd() const override { return fd_; }

  void OnInputEvent() override;

 private:
  std::string device_;
  int fd_;
  std::array<JsAxisValue, static_cast<int>(JsAxis::kLast)> axis_info_;
  std::vector<JoystickButtonEventCallback> button_event_callbacks_;
  std::vector<JoystickAxisEventCallback> axis_event_callbacks_;
};
}  // namespace xmotion

#endif  // XMOTION_JOYSTICK_HANDLER_HPP