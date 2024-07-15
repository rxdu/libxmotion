/*
 * joystick.hpp
 *
 * Created on 5/30/23 11:00 PM
 * Description: A joystick class that monitors joystick events using polling way
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
#include <mutex>

#include "interface/driver/joystick_interface.hpp"

namespace xmotion {
class Joystick : public JoystickInterface {
 public:
  static constexpr unsigned int max_js_buttons = 32;
  static constexpr unsigned int max_js_axes = 32;

  static std::vector<JoystickDescriptor> EnumberateJoysticks(
      int max_index = 32);

 public:
  Joystick();
  explicit Joystick(JoystickDescriptor descriptor);
  explicit Joystick(int index);
  explicit Joystick(const std::string& event_name);
  ~Joystick();

  // do not allow copy
  Joystick(const Joystick&) = delete;
  Joystick& operator=(const Joystick&) = delete;

  // public methods
  std::string GetDeviceName() const override { return descriptor_.name; };

  int GetDeviceIndex() const { return descriptor_.index; };

  bool Open() override;
  void Close() override;
  bool IsOpened() const override;

  std::string GetButtonName(const JsButton& btn) const;
  std::string GetAxisName(const JsAxis& axis) const;

  bool GetButtonState(const JsButton& btn) const override;
  JsAxisValue GetAxisState(const JsAxis& axis) const override;

  void SetJoystickRumble(short weakRumble, short strongRumble);

  void PollEvent() override;

 private:
  void InitializeChannels();
  void ReadJoystickInput();

  JoystickDescriptor descriptor_;
  std::string device_name_;

  int fd_ = -1;
  int device_change_notify_;
  std::thread io_thread_;
  std::atomic<bool> keep_running_{false};
  std::atomic<bool> connected_{false};

  mutable std::mutex buttons_mtx_;
  bool buttons_[max_js_buttons];

  mutable std::mutex axes_mtx_;
  JsAxisValue axes_[max_js_axes];

  bool has_rumble_ = false;
  short rumble_effect_id_;
};
}  // namespace xmotion

#endif  // ROBOSW_JOYSTICK_HPP
