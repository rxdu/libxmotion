/*
 * utest_config_loader.cpp
 *
 * Created on 7/9/24 11:23 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include "quadruped/config_loader.hpp"

using namespace xmotion;

TEST(ConfigLoaderTest, LoadConfigFileTest) {
  SystemConfig config;
  ConfigLoader::LoadConfigFile("../config/go2_sim.yaml", &config);

  EXPECT_EQ(config.dds_domain_id, 1);
  EXPECT_EQ(config.network_interface, "lo");
  EXPECT_EQ(config.is_simulation, true);

  EXPECT_EQ(config.hid_settings.keyboard.enable, true);
  EXPECT_EQ(config.hid_settings.keyboard.device, "/dev/input/event3");
  EXPECT_EQ(config.hid_settings.keyboard.keyboard_mappings[KeyboardCode::kF1],
            HidSettings::KeyFunction::kPassiveMode);
  EXPECT_EQ(config.hid_settings.keyboard.keyboard_mappings[KeyboardCode::kF2],
            HidSettings::KeyFunction::kFixedStandMode);
  EXPECT_EQ(config.hid_settings.keyboard.keyboard_mappings[KeyboardCode::kF3],
            HidSettings::KeyFunction::kFreeStandMode);
  EXPECT_EQ(config.hid_settings.keyboard.keyboard_mappings[KeyboardCode::kF4],
            HidSettings::KeyFunction::kTrottingMode);
  EXPECT_EQ(config.hid_settings.keyboard.keyboard_mappings[KeyboardCode::kF5],
            HidSettings::KeyFunction::kMoveBaseMode);


}