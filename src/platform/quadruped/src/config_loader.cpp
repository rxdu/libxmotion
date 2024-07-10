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

#include "input_hid/keyboard.hpp"
#include "logging/xlogger.hpp"

namespace xmotion {
namespace {
std::unordered_map<std::string, HidSettings::KeyFunction> str_to_key_mapping = {
    {"passive_mode", HidSettings::KeyFunction::kPassiveMode},
    {"fixed_stand_mode", HidSettings::KeyFunction::kFixedStandMode},
    {"free_stand_mode", HidSettings::KeyFunction::kFreeStandMode},
    {"trotting_mode", HidSettings::KeyFunction::kTrottingMode},
    {"move_base_mode", HidSettings::KeyFunction::kMoveBaseMode}};
}

bool ConfigLoader::LoadConfigFile(const std::string &file_path,
                                  SystemConfig *config) {
  try {
    YAML::Node config_node = YAML::LoadFile(file_path);

    config->dds_domain_id = config_node["dds_domain_id"].as<int>();
    config->network_interface =
        config_node["network_interface"].as<std::string>();
    config->is_simulation = config_node["is_simulation"].as<bool>();

    /*------------------------------------------------------------------------*/
    // HID settings
    /*------------------------------------------------------------------------*/

    config->hid_settings.keyboard.enable =
        config_node["hid_settings"]["keyboard"]["enable"].as<bool>();
    config->hid_settings.keyboard.device =
        config_node["hid_settings"]["keyboard"]["device_name"]
            .as<std::string>();
    for (auto it = config_node["hid_settings"]["keyboard"]["keyboard_mappings"]
                       .begin();
         it !=
         config_node["hid_settings"]["keyboard"]["keyboard_mappings"].end();
         ++it) {
      if (str_to_key_mapping.find(it->first.as<std::string>()) ==
          str_to_key_mapping.end()) {
        XLOG_ERROR("ConfigLoader: unknown key mapping: {}",
                   it->first.as<std::string>());
        return false;
      }
      auto key_code = Keyboard::GetKeyCode(it->second.as<std::string>());
      if (key_code == KeyboardCode::kUnknown) {
        XLOG_ERROR("ConfigLoader: unknown key code: {}",
                   it->second.as<std::string>());
        return false;
      }
      config->hid_settings.keyboard.keyboard_mappings[Keyboard::GetKeyCode(
          it->second.as<std::string>())] =
          str_to_key_mapping[it->first.as<std::string>()];
      XLOG_INFO("ConfigLoader: key mapping: {} -> {}",
                it->first.as<std::string>(), it->second.as<std::string>());
    }

    config->hid_settings.joystick.enable =
        config_node["hid_settings"]["joystick"]["enable"].as<bool>();
    config->hid_settings.joystick.device =
        config_node["hid_settings"]["joystick"]["device_name"]
            .as<std::string>();

    /*------------------------------------------------------------------------*/
    // control settings
    /*------------------------------------------------------------------------*/
    for (int i = 0; i < 12; ++i) {
      config->ctrl_settings.passive_mode.joint_gains.kp[i] =
          config_node["control_settings"]["passive_mode"]["joint_gains"]
                     ["joint" + std::to_string(i)]["kp"]
                         .as<double>();
      config->ctrl_settings.passive_mode.joint_gains.kd[i] =
          config_node["control_settings"]["passive_mode"]["joint_gains"]
                     ["joint" + std::to_string(i)]["kd"]
                         .as<double>();
    }

  } catch (YAML::BadFile &e) {
    XLOG_ERROR("ConfigLoader: failed to open config file {}: {}", file_path,
               e.what());
    return false;
  }
  return true;
}
}  // namespace xmotion