/*
 * app_quadruped_locomotion.cpp
 *
 * Created on 7/6/24 7:50 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <csignal>
#include <atomic>
#include <memory>

#include "logging/xlogger.hpp"

#include "quadruped/config_loader.hpp"
#include "quadruped/robot_model/unitree_model_profile.hpp"
#include "quadruped/robot_model/unitree_dog.hpp"
#include "quadruped/quadruped_system.hpp"

using namespace xmotion;

std::unique_ptr<QuadrupedSystem> quadruped;

int main(int argc, char **argv) {
  // register signal handler
  std::signal(SIGINT, [](int signal) -> void {
    if (signal == SIGINT) {
      if (quadruped != nullptr) {
        XLOG_INFO("Received SIGINT");
        quadruped->Stop();
      }
    }
  });

  // handle command line arguments
  std::string config_file_path = "../config/go2_sim.yaml";
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <config_file_path>" << std::endl;
    return -1;
  }
  config_file_path = argv[1];

  XLOG_INFO("Starting Quadruped Locomotion Application");

  // load configuration
  SystemConfig config;
  if (!ConfigLoader::LoadConfigFile(config_file_path, &config)) {
    XLOG_ERROR("Failed to load configuration file {}", config_file_path);
    return -1;
  }
  XLOG_INFO("Loaded configuration file {}", config_file_path);

  // create a robot model
  auto dog_model = std::make_shared<UnitreeDog>(config.dds_domain_id,
                                                config.network_interface,
                                                UnitreeDogs::GetGo2Profile());

  // create a quadruped system and initialize the components
  quadruped = std::make_unique<QuadrupedSystem>(config, dog_model);

  if (!quadruped->Initialize()) {
    XLOG_ERROR("Failed to initialize quadruped system, exiting...");
    return -1;
  }

  quadruped->Run();

  XLOG_INFO("Quadruped Locomotion Application Exited");

  return 0;
}