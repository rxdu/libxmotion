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
    kFreeStandMode,
    kTrottingMode,
    kSwingTestMode,
    kBalanceTestMode,
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
  std::unordered_map<std::string, QuadrupedModel::AllJointGains> gain_sets;

  struct PassiveModeParams {
    QuadrupedModel::AllJointGains default_joint_gains;
  } passive_mode;

  struct FixedStandModeParams {
    QuadrupedModel::AllJointGains default_joint_gains;
    QuadrupedModel::AllJointVar desired_joint_position;
    uint32_t duration_ms;
  } fixed_stand_mode;

  struct LyingDownModeParams {
    QuadrupedModel::AllJointGains default_joint_gains;
    QuadrupedModel::AllJointVar desired_joint_position;
    uint32_t duration_ms;
  } lying_down_mode;

  struct FreeStandModeParams {
    QuadrupedModel::AllJointGains default_joint_gains;

    struct PoseLimit {
      double roll;
      double pitch;
      double yaw;
      double height;
    } pose_limit;

    double angle_step;
    double height_step;
  } free_stand_mode;

  struct SwingTestModeParams {
    QuadrupedModel::AllJointGains default_joint_gains;
    LegIndex swing_leg_index;
    QuadrupedModel::LegJointGains swing_leg_gains;

    Eigen::Matrix<double, 3, 1> kp;
    Eigen::Matrix<double, 3, 1> kd;

    struct ChangeLimit {
      double x_min;
      double x_max;
      double y_min;
      double y_max;
      double z_min;
      double z_max;
    } change_limit;

    double move_step;
  } swing_test_mode;

  struct BalanceTestModeParams {
    QuadrupedModel::AllJointGains default_joint_gains;

    struct PositionController {
      Eigen::Matrix<double, 3, 1> kp;
      Eigen::Matrix<double, 3, 1> kd;
    } position_controller;

    struct OrientationController {
      double kp;
      Eigen::Matrix<double, 3, 1> kd;
    } orientation_controller;

    struct PoseLimit {
      double x;
      double y;
      double z;
      double yaw;
    } pose_limit;

    double move_step;
    double rotate_step;
  } balance_test_mode;
};

struct EstimatorSettings {
  double expected_dt;

  struct SimpleEstimatorParams {
    Eigen::Matrix<double, 18, 1> Q_diag;
  } simple_estimator;
};

struct SystemConfig {
  int dds_domain_id;
  std::string network_interface;
  bool is_simulation;

  HidSettings hid_settings;
  ControlSettings ctrl_settings;
  EstimatorSettings est_settings;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_SYSTEM_CONFIG_HPP
