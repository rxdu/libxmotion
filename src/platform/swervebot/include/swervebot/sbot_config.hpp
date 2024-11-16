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
  };

  BaseSettings base_settings;

  struct {
    std::string joystick_device;
  } hid_settings;
};

bool LoadConfigFile(const std::string &file_path, SbotConfig *config);
}  // namespace xmotion

#endif  // XMOTION_SBOT_CONFIG_HPP