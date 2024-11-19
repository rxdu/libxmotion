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
    config->base_settings.max_driving_speed =
        config_node["robot_base"]["max_driving_speed"].as<float>();
    config->base_settings.max_steering_angle =
        config_node["robot_base"]["max_steering_angle"].as<float>();
    config->base_settings.driving_deadzone =
        config_node["robot_base"]["driving_deadzone"].as<float>();
    config->base_settings.steering_deadzone =
        config_node["robot_base"]["steering_deadzone"].as<float>();

    // HID settings
    config->hid_settings.joystick_device =
        config_node["hid"]["joystick_device"].as<std::string>();
    config->hid_settings.sbus_port =
        config_node["hid"]["sbus_port"].as<std::string>();

    // control settings
    std::string input_type =
        config_node["control"]["input_type"].as<std::string>();
    if (input_type == "joystick") {
      config->control_settings.input_type =
          SbotConfig::ControlInputType::kJoystick;
    } else if (input_type == "sbus") {
      config->control_settings.input_type = SbotConfig::ControlInputType::kSbus;
    } else {
      config->control_settings.input_type = SbotConfig::ControlInputType::kNone;
    }

    config->control_settings.manual_mode.driving_scale =
        config_node["control"]["manual_mode"]["driving_scale"].as<float>();
    config->control_settings.manual_mode.steering_scale =
        config_node["control"]["manual_mode"]["steering_scale"].as<float>();

    config->control_settings.auto_mode.driving_scale =
        config_node["control"]["auto_mode"]["driving_scale"].as<float>();
    config->control_settings.auto_mode.steering_scale =
        config_node["control"]["auto_mode"]["steering_scale"].as<float>();

  } catch (YAML::BadFile& e) {
    XLOG_ERROR("ConfigLoader: failed to open config file {}: {}", file_path,
               e.what());
    return false;
  }
  return true;
}
}  // namespace xmotion