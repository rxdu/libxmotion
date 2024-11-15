/*
 * @file app_ws_sbot_controller.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <csignal>
#include <thread>
#include <chrono>
#include <iostream>

#include "logging/xlogger.hpp"
#include "swervebot/ws_sbot_base.hpp"
#include "swervebot/sbot_system.hpp"

using namespace xmotion;

bool keep_running = true;
std::unique_ptr<SbotSystem> sbot_system;

int main(int argc, char* argv[]) {
  // register signal handler
  std::signal(SIGINT, [](int signal) -> void {
    if (signal == SIGINT) {
      if (sbot_system != nullptr) {
        XLOG_INFO("Received SIGINT, stopping sbot system...");
        sbot_system->Stop();
      }
    }
  });

  std::string config_file = "../src/platform/swervebot/config/sbot.yaml";
  if (argc == 2) {
    config_file = argv[1];
  } else {
    std::cout << "Usage: " << argv[0] << " <config_file>" << std::endl;
    return -1;
  }

  SbotConfig config;
  if (!LoadConfigFile(config_file, &config)) {
    XLOG_ERROR("Failed to load configuration file {}", config_file);
    return -1;
  }

  sbot_system = std::make_unique<SbotSystem>(config);

  if (!sbot_system->Initialize()) {
    XLOG_ERROR("Failed to initialize sbot system, exiting...");
    return -1;
  }

  sbot_system->Run();

  XLOG_INFO("Sbot Controller Application Exited");

  return 0;
}