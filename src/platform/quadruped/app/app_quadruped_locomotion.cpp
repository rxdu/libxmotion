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

#include "quadruped/robot_model/unitree_model_profile.hpp"
#include "quadruped/robot_model/unitree_dog.hpp"
#include "quadruped/quadruped_system.hpp"

using namespace xmotion;

std::unique_ptr<QuadrupedSystem> quadruped;

int main(int argc, char **argv) {
  // register signal handler
  std::signal(SIGINT, [](int signal) -> void {
    if (signal == SIGINT) {
      if (quadruped != nullptr) quadruped->Stop();
    }
  });

  XLOG_INFO("Starting Quadruped Locomotion Application");

  // create a robot model
  auto dog_model = std::make_shared<UnitreeDog>(UnitreeDogs::GetGo2Profile());

  // create a quadruped system and initialize the components
  quadruped = std::make_unique<QuadrupedSystem>(dog_model);
  
  if (!quadruped->Initialize()) {
    XLOG_ERROR("Failed to initialize quadruped system, exiting...");
    return -1;
  }

  quadruped->Run();

  return 0;
}