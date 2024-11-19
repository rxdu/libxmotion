/*
 * @file sbot_config.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SBOT_CONFIG_HPP
#define XMOTION_SBOT_CONFIG_HPP

#include <string>

namespace xmotion {
struct SbotConfig {
  struct BaseSettings {
    std::string steering_motor_port;
    std::string driving_motor_port;
    float max_driving_speed;
    float max_steering_angle;
    float driving_deadzone;
    float steering_deadzone;
  };

  enum class ControlInputType {
    kNone,
    kJoystick,
    kSbus,
  };

  struct {
    std::string joystick_device;
    std::string sbus_port;
  } hid_settings;

  BaseSettings base_settings;

  struct {
    ControlInputType input_type;

    struct {
      float driving_scale;
      float steering_scale;
    } manual_mode;

    struct {
      float driving_scale;
      float steering_scale;
    } auto_mode;
  } control_settings;
};

bool LoadConfigFile(const std::string &file_path, SbotConfig *config);
}  // namespace xmotion

#endif  // XMOTION_SBOT_CONFIG_HPP