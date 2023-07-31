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

namespace xmotion {
class JoystickTeleop {
 public:
  struct Config {
    JoystickDescriptor jd;
  };

 public:
  JoystickTeleop(const Config& config);
  ~JoystickTeleop();

  bool Initialize();
  void Run(int rate_hz);

 private:
  Config config_;
  std::unique_ptr<JoystickInterface> joystick_;
};
}  // namespace xmotion

#endif  // XMOTION_LIB_JOYSTICK_TELEOP_HPP
