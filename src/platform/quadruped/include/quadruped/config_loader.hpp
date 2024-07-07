/*
 * @file config_loader.hpp
 * @date 7/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_CONFIG_LOADER_HPP
#define QUADRUPED_MOTION_CONFIG_LOADER_HPP

#include <string>

namespace xmotion {
struct RobotConfig {
  int dds_domain_id;
  std::string network_interface;
};

struct ConfigLoader {
  static bool LoadConfigFile(const std::string& file_path, RobotConfig* config);
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_CONFIG_LOADER_HPP
