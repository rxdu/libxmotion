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
QuadrupedSystem::QuadrupedSystem(const SystemConfig& config,
                                 std::shared_ptr<QuadrupedModel> model)
    : config_(config), model_(model) {
  hid_event_listener_ = std::make_shared<HidEventHandler>(config_.hid_config);
}

QuadrupedSystem::~QuadrupedSystem() { Stop(); }

bool QuadrupedSystem::Initialize() {
  XLOG_INFO("QuadrupedSystem: initializing...");

  // initialize event listeners
  if (!hid_event_listener_->Initialize()) return false;

  // initialize robot mode if not simulation
  if (!config_.is_simulation) {
    // TODO initialize robot mode
  }

  // initialize estimator subsystem

  // initialize control subsystem
  PassiveMode initial_state;
  ControlContext context;
  context.system_config = config_;
  context.robot_model = model_;
  context.hid_event_listener = hid_event_listener_;
  fsm_ = std::make_unique<ControlModeFsm>(std::move(initial_state),
                                          std::move(context));
  keep_control_loop_ = true;

  XLOG_INFO("====== QuadrupedSystem: initialized ======");

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
  // start estimator thread

  // start control thread
  control_thread_ = std::thread(&QuadrupedSystem::ControlSubsystem, this);

  // at last start the event listener
  hid_event_listener_->Start();

  // start main loop for housekeeping
  XLOG_INFO("QuadrupedSystem: entering main loop");
  keep_running_ = true;
  while (keep_running_) {
    hid_event_listener_->PollEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  XLOG_INFO("QuadrupedSystem: main loop exited");
}

void QuadrupedSystem::Stop() {
  XLOG_INFO("Stopping QuadrupedSystem");

  // wait for control thread to finish
  keep_control_loop_ = false;
  if (control_thread_.joinable()) control_thread_.join();

  // finally stop the main loop
  keep_running_ = false;
}
}  // namespace xmotion