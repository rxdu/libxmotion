/*
 * joystick_teleop.hpp
 *
 * Created on 6/6/23 8:54 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef XMOTION_LIB_JOYSTICK_TELEOP_HPP
#define XMOTION_LIB_JOYSTICK_TELEOP_HPP

#include <memory>

#include "interface/driver/joystick_interface.hpp"
#include "motor_vesc/vesc_can_interface.hpp"



namespace xmotion {
class JoystickTeleop {
  enum class ControlSource : int { kNone = 0, kJoystick, kCommand };

 public:
  struct Config {
    JoystickDescriptor jd;
    std::string can_if_name;
    uint8_t vesc_id;
  };

 public:
  JoystickTeleop(const Config& config);
  ~JoystickTeleop();

  bool Initialize();
  void Run(int rate_hz);

 private:
  void VescStateUpdatedCallback(const StampedVescState& state);

  Config config_;
  std::unique_ptr<JoystickInterface> joystick_;
  std::unique_ptr<VescCanInterface> vesc_;

  ControlSource control_source_ = ControlSource::kNone;
};
}  // namespace xmotion

#endif  // XMOTION_LIB_JOYSTICK_TELEOP_HPP
