/*
 * joystick.hpp
 *
 * Created on 5/30/23 11:00 PM
 * Description:
 *
 * Reference: https://github.com/MysteriousJ/Joystick-Input-Examples
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_JOYSTICK_HPP
#define ROBOSW_JOYSTICK_HPP

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>

#include "interface/driver/joystick_interface.hpp"

namespace robosw {
class Joystick : public JoystickInterface {
 public:
  static constexpr unsigned int max_js_buttons = 32;
  static constexpr unsigned int max_js_axes = 32;

  static std::vector<JoystickDescriptor> EnumberateJoysticks(
      int max_index = 32);

 public:
  Joystick(JoystickDescriptor descriptor);
  ~Joystick();

  std::string GetDeviceName() const override { return descriptor_.name; };
  int GetDeviceIndex() const { return descriptor_.index; };

  bool Open() override;
  void Close() override;
  bool IsOpened() const override;

  void SetButtonEventCallback(ButtonEventCallback cb) override;
  void SetAxisEventCallback(AxisEventCallback cb) override;

 private:
  void ReadJoystickInput();
  void Update();

  JoystickDescriptor descriptor_;

  int fd_ = -1;
  int device_change_notify_;
  std::thread io_thread_;
  std::atomic<bool> keep_running_{false};
  std::atomic<bool> connected_{false};

  bool buttons_[max_js_buttons];
  JsAxisValue axes_[max_js_axes];

  bool has_rumble_ = false;
  short rumble_effect_id_;
};
}  // namespace robosw

#endif  // ROBOSW_JOYSTICK_HPP
