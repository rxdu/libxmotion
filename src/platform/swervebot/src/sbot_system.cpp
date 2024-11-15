/*
 * @file sbot_system.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/sbot_system.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
SbotSystem::SbotSystem(const SbotConfig& config) : config_(config) {}

bool SbotSystem::Initialize() {
  sbot_ = std::make_shared<WsSbotBase>(config_.base_config);
  if (!sbot_->Initialize()) {
    XLOG_ERROR("Failed to initialize robot base");
    return false;
  }

  // initialize fsm
  ControlContext context;
  context.config = config_;
  context.robot_base = sbot_;

  ManualMode initial_state{context};
  fsm_ =
      std::make_unique<SbotFsm>(std::move(initial_state), std::move(context));

  return true;
}

void SbotSystem::Run() {
  XLOG_INFO("SbotSystem: entering main loop");
  keep_running_ = true;
  while (keep_running_) {
    // handle user input logic here
    sbot_->SetDrivingCommand({0.2, 0.2, 0.2, 0.2});
    float angle = 0;
    sbot_->SetSteeringCommand({angle, angle, angle, angle});
    std::this_thread::sleep_for(std::chrono::seconds(2));

    fsm_->Update();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  XLOG_INFO("SbotSystem: main loop exited");
}

void SbotSystem::Stop() {
  // stop the robot
  sbot_->SetSteeringCommand({0, 0, 0, 0});
  sbot_->SetDrivingCommand({0, 0, 0, 0});

  // finally stop the main loop
  keep_running_ = false;
}
}  // namespace xmotion