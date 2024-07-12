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
    {"swing_test_mode", HidSettings::KeyFunction::kSwingTestMode},
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
    for (auto it = config_node["control_settings"]["gain_sets"].begin();
         it != config_node["control_settings"]["gain_sets"].end(); ++it) {
      QuadrupedModel::JointGains gains;
      for (int i = 0; i < 12; ++i) {
        gains.kp[i] =
            it->second["joint" + std::to_string(i)]["kp"].as<double>();
        gains.kd[i] =
            it->second["joint" + std::to_string(i)]["kd"].as<double>();
      }
      config->ctrl_settings.gain_sets[it->first.as<std::string>()] = gains;
    }

    // passive mode default joint gains
    {
      std::string gain_set_name =
          config_node["control_settings"]["passive_mode"]["default_joint_gains"]
              .as<std::string>();
      if (config->ctrl_settings.gain_sets.find(gain_set_name) ==
          config->ctrl_settings.gain_sets.end()) {
        XLOG_ERROR("ConfigLoader: passive mode default gain set not found: {}",
                   gain_set_name);
        return false;
      }
      config->ctrl_settings.passive_mode.default_joint_gains =
          config->ctrl_settings.gain_sets[gain_set_name];
    }

    // fixed stand mode desired joint position
    {
      std::string gain_set_name =
          config_node["control_settings"]["fixed_stand_mode"]
                     ["default_joint_gains"]
                         .as<std::string>();
      if (config->ctrl_settings.gain_sets.find(gain_set_name) ==
          config->ctrl_settings.gain_sets.end()) {
        XLOG_ERROR(
            "ConfigLoader: fixed stand mode default gain set not found: {}",
            gain_set_name);
        return false;
      }
      config->ctrl_settings.fixed_stand_mode.default_joint_gains =
          config->ctrl_settings.gain_sets[gain_set_name];

      std::vector<double> q_desired =
          config_node["control_settings"]["fixed_stand_mode"]["q_desired"]
              .as<std::vector<double>>();
      if (q_desired.size() != 12) {
        XLOG_ERROR(
            "ConfigLoader: fixed stand mode desired joint position size "
            "mismatch");
        return false;
      }
      for (size_t i = 0; i < q_desired.size(); ++i) {
        config->ctrl_settings.fixed_stand_mode.desired_joint_position(i) =
            q_desired[i];
      }

      config->ctrl_settings.fixed_stand_mode.duration_ms =
          config_node["control_settings"]["fixed_stand_mode"]["duration_ms"]
              .as<uint32_t>();
    }

    // lying down mode desired joint position
    {
      std::string gain_set_name =
          config_node["control_settings"]["lying_down_mode"]
                     ["default_joint_gains"]
                         .as<std::string>();
      if (config->ctrl_settings.gain_sets.find(gain_set_name) ==
          config->ctrl_settings.gain_sets.end()) {
        XLOG_ERROR(
            "ConfigLoader: fixed stand mode default gain set not found: {}",
            gain_set_name);
        return false;
      }
      config->ctrl_settings.lying_down_mode.default_joint_gains =
          config->ctrl_settings.gain_sets[gain_set_name];

      std::vector<double> q_desired =
          config_node["control_settings"]["lying_down_mode"]["q_desired"]
              .as<std::vector<double>>();
      if (q_desired.size() != 12) {
        XLOG_ERROR(
            "ConfigLoader: lying down mode desired joint position size "
            "mismatch");
        return false;
      }
      for (size_t i = 0; i < q_desired.size(); ++i) {
        config->ctrl_settings.lying_down_mode.desired_joint_position(i) =
            q_desired[i];
      }

      config->ctrl_settings.lying_down_mode.duration_ms =
          config_node["control_settings"]["lying_down_mode"]["duration_ms"]
              .as<uint32_t>();
    }

    // swing test mode default joint gains
    {
      std::string gain_set_name =
          config_node["control_settings"]["swing_test_mode"]
                     ["default_joint_gains"]
                         .as<std::string>();
      if (config->ctrl_settings.gain_sets.find(gain_set_name) ==
          config->ctrl_settings.gain_sets.end()) {
        XLOG_ERROR(
            "ConfigLoader: swing test mode default gain set not found: {}",
            gain_set_name);
        return false;
      }
      config->ctrl_settings.swing_test_mode.default_joint_gains =
          config->ctrl_settings.gain_sets[gain_set_name];
    }
  } catch (YAML::BadFile &e) {
    XLOG_ERROR("ConfigLoader: failed to open config file {}: {}", file_path,
               e.what());
    return false;
  }
  return true;
}
}  // namespace xmotion