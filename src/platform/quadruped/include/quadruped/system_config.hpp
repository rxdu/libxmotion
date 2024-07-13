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
#include <unordered_map>

#include "interface/driver/keyboard_interface.hpp"
#include "quadruped/robot_model/quadruped_model.hpp"

namespace xmotion {
struct HidSettings {
  enum class KeyFunction : int {
    // keys for mode selection
    kFirstModeKey = 0,
    kPassiveMode,
    kFixedStandMode,
    kSwingTestMode,
    kFreeStandMode,
    kTrottingMode,
    kMoveBaseMode,
    kLastModeKey,
    // keys for control
    kFirstControlKey,
    kLeftStickUp,
    kLeftStickDown,
    kLeftStickLeft,
    kLeftStickRight,
    kRightStickUp,
    kRightStickDown,
    kRightStickLeft,
    kRightStickRight,
    kLastControlKey
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
  std::unordered_map<std::string, QuadrupedModel::JointGains> gain_sets;

  struct PassiveModeParams {
    QuadrupedModel::JointGains default_joint_gains;
  } passive_mode;

  struct FixedStandModeParams {
    QuadrupedModel::JointGains default_joint_gains;
    QuadrupedModel::JointVar desired_joint_position;
    uint32_t duration_ms;
  } fixed_stand_mode;

  struct LyingDownModeParams {
    QuadrupedModel::JointGains default_joint_gains;
    QuadrupedModel::JointVar desired_joint_position;
    uint32_t duration_ms;
  } lying_down_mode;

  struct SwingTestModeParams {
    QuadrupedModel::JointGains default_joint_gains;
    LegIndex swing_leg_index;
    Eigen::Matrix<double, 3, 1> kp;
    Eigen::Matrix<double, 3, 1> kd;

    struct Range {
      double x_min;
      double x_max;
      double y_min;
      double y_max;
      double z_min;
      double z_max;
    } range;
  } swing_test_mode;
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
