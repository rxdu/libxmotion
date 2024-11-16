/*
 * @file sbot_config.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/sbot_config.hpp"

#include "yaml-cpp/yaml.h"

#include "logging/xlogger.hpp"

namespace xmotion {
bool LoadConfigFile(const std::string& file_path, SbotConfig* config) {
  try {
    YAML::Node config_node = YAML::LoadFile(file_path);

    // base config
    config->base_settings.steering_motor_port =
        config_node["robot_base"]["steering_motor_port"].as<std::string>();
    config->base_settings.driving_motor_port =
        config_node["robot_base"]["driving_motor_port"].as<std::string>();

    // HID settings
    config->hid_settings.joystick_device =
        config_node["hid"]["joystick_device"].as<std::string>();

  } catch (YAML::BadFile& e) {
    XLOG_ERROR("ConfigLoader: failed to open config file {}: {}", file_path,
               e.what());
    return false;
  }
  return true;
}
}  // namespace xmotion