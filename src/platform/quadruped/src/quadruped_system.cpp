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
  hid_event_listener_ = std::make_shared<HidEventHandler>(config_.hid_settings);
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
  estimator_ = std::make_unique<SimpleEstimator>(config_.est_settings, model_);
  SimpleEstimator::Params estimator_params;
  estimator_->Initialize(estimator_params);
  keep_estimation_loop_ = true;

  // initialize control subsystem
  ControlContext context;
  context.system_config = config_;
  context.robot_model = model_;
  sensor_data_queue_ =
      std::make_shared<DataQueue<QuadrupedModel::SensorData>>();
  context.robot_model->ConnectSensorDataQueue(sensor_data_queue_);
  context.hid_event_listener = hid_event_listener_;
  PassiveMode initial_state{context};
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
    timer.sleep_until_ms(2);
  }
  XLOG_INFO("QuadrupedSystem: control loop exited");
}

void QuadrupedSystem::EstimationSubsystem() {
  XLOG_INFO("QuadrupedSystem: entering estimation loop");
  //  Timer timer;
  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  TimePoint last_time = Clock::now();
  bool first_run = true;
  while (keep_estimation_loop_) {
    //    timer.reset();
    QuadrupedModel::SensorData data;
    if (sensor_data_queue_->Pop(data)) {
      if (first_run) {
        last_time = Clock::now();
        first_run = false;
      }
      auto now = Clock::now();
      double dt = std::chrono::duration<double>(now - last_time).count();
      last_time = now;
      estimator_->Update(data, dt);
    }
    //    timer.sleep_until_ms(2);
  }
  XLOG_INFO("QuadrupedSystem: estimation loop exited");
}

void QuadrupedSystem::Run() {
  // start estimator thread
  estimation_thread_ = std::thread(&QuadrupedSystem::EstimationSubsystem, this);

  // start control thread
  control_thread_ = std::thread(&QuadrupedSystem::ControlSubsystem, this);

  // at last start the event listener
  hid_event_listener_->Start();

  // start main loop for housekeeping
  XLOG_INFO("QuadrupedSystem: entering main loop");
  keep_running_ = true;
  //  StopWatch sw;
  while (keep_running_) {
    //    if (sw.stoc() > 3) {
    //      XLOG_INFO("QuadrupedSystem: main loop running");
    //      break;
    //    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  XLOG_INFO("QuadrupedSystem: main loop exited");
}

void QuadrupedSystem::Stop() {
  // DO NOT CALL ANY XLOG BEYOND THIS LINE AS A SIGNAL MAY HAVE BEEN TRIGGERED
  // wait for control thread to finish
  keep_control_loop_ = false;
  if (control_thread_.joinable()) control_thread_.join();

  // wait for estimation thread to finish
  keep_estimation_loop_ = false;
  if (estimation_thread_.joinable()) estimation_thread_.join();

  // finally stop the main loop
  keep_running_ = false;
}
}  // namespace xmotion