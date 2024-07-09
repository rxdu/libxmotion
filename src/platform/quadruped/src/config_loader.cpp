/*
 * config_loader.cpp
 *
 * Created on 7/7/24 11:57 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/config_loader.hpp"

#include "yaml-cpp/yaml.h"

#include "logging/xlogger.hpp"

namespace xmotion {
bool ConfigLoader::LoadConfigFile(const std::string &file_path,
                                  SystemConfig *config) {
  try {
    YAML::Node config_node = YAML::LoadFile(file_path);

    config->dds_domain_id = config_node["dds_domain_id"].as<int>();
    config->network_interface =
        config_node["network_interface"].as<std::string>();
    config->is_simulation = config_node["is_simulation"].as<bool>();

    // HID config
    config->hid_config.keyboard.enable =
        config_node["hid_config"]["keyboard"]["enable"].as<bool>();
    config->hid_config.keyboard.device =
        config_node["hid_config"]["keyboard"]["device_name"].as<std::string>();

    config->hid_config.joystick.enable =
        config_node["hid_config"]["joystick"]["enable"].as<bool>();
    config->hid_config.joystick.device =
        config_node["hid_config"]["joystick"]["device_name"].as<std::string>();
  } catch (YAML::BadFile &e) {
    XLOG_ERROR("ConfigLoader: failed to open config file {}: {}", file_path,
               e.what());
    return false;
  }
  return true;
}
}  // namespace xmotion