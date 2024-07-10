/*
 * system_config.hpp
 *
 * Created on 7/8/24 8:46 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_SYSTEM_CONFIG_HPP
#define QUADRUPED_MOTION_SYSTEM_CONFIG_HPP

#include <string>

#include "interface/driver/keyboard_interface.hpp"
#include "quadruped/robot_model/quadruped_model.hpp"

namespace xmotion {
struct HidSettings {
  enum class KeyFunction : int {
    kPassiveMode = 0,
    kFixedStandMode,
    kFreeStandMode,
    kTrottingMode,
    kMoveBaseMode,
  };

  struct Keyboard {
    bool enable;
    std::string device;
    std::unordered_map<KeyboardCode, KeyFunction> keyboard_mappings;
  };

  struct Joystick {
    bool enable;
    std::string device;
  };

  Keyboard keyboard;
  Joystick joystick;
};

struct ControlSettings {
  struct PassiveModeParams {
    QuadrupedModel::JointGains joint_gains;
  } passive_mode;
};

struct SystemConfig {
  int dds_domain_id;
  std::string network_interface;
  bool is_simulation;

  HidSettings hid_settings;
  ControlSettings ctrl_settings;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_SYSTEM_CONFIG_HPP
