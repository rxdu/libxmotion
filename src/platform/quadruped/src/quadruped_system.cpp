/*
 * quadruped_system.cpp
 *
 * Created on 7/6/24 10:59 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/quadruped_system.hpp"

#include "time/stopwatch.hpp"
#include "logging/xlogger.hpp"

namespace xmotion {
QuadrupedSystem::QuadrupedSystem(std::shared_ptr<QuadrupedModel> model)
    : model_(model) {}

QuadrupedSystem::~QuadrupedSystem() {
  keep_running_ = false;

  // wait for control thread to finish
  keep_control_loop_ = false;
  if (control_thread_.joinable()) control_thread_.join();
}

bool QuadrupedSystem::Initialize() {
  // initialize estimator subsystem

  // initialize control subsystem
  PassiveMode initial_state;
  ControlContext context;
  fsm_ = std::make_unique<ControlModeFsm>(std::move(initial_state),
                                          std::move(context));
  keep_control_loop_ = true;

  return true;
}

void QuadrupedSystem::ControlSubsystem() {
  XLOG_INFO("QuadrupedSystem: entering control loop");
  Timer timer;
  while (keep_control_loop_) {
    timer.reset();
    fsm_->Update();
    //    timer.sleep_until_us(2000);
    timer.sleep_until_ms(1000);
  }
  XLOG_INFO("QuadrupedSystem: control loop exited");
}

void QuadrupedSystem::Run() {
  control_thread_ = std::thread(&QuadrupedSystem::ControlSubsystem, this);

  // main loop for housekeeping
  XLOG_INFO("QuadrupedSystem: entering main loop");
  keep_running_ = true;
  while (keep_running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  XLOG_INFO("QuadrupedSystem: main loop exited");
}

void QuadrupedSystem::Stop() {
  XLOG_INFO("Stopping QuadrupedSystem");
  keep_running_ = false;
}
}  // namespace xmotion