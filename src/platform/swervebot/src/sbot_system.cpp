/*
 * @file sbot_system.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/sbot_system.hpp"

#include "stopwatch/stopwatch.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
SbotSystem::SbotSystem(const SbotConfig& config) : config_(config) {}

bool SbotSystem::Initialize() {
  // initialize joystick
  joystick_ =
      std::make_unique<JoystickHandler>(config_.hid_settings.joystick_device);
  if (!joystick_->Open()) {
    XLOG_ERROR("Failed to open joystick device");
    return false;
  }
  joystick_->RegisterJoystickButtonEventCallback(
      std::bind(&SbotSystem::OnJsButtonEvent, this, std::placeholders::_1,
                std::placeholders::_2));
  joystick_->RegisterJoystickAxisEventCallback(
      std::bind(&SbotSystem::OnJsAxisEvent, this, std::placeholders::_1,
                std::placeholders::_2));

  hid_event_listener_ = std::make_shared<HidEventListener>();
  if (!hid_event_listener_->AddHidHandler(joystick_.get())) {
    XLOG_ERROR("Failed to add joystick handler to event listener");
    return false;
  }
  hid_event_listener_->StartListening();

  // initialize robot base
  sbot_ = std::make_shared<WsSbotBase>(config_.base_settings);
  if (!sbot_->Initialize()) {
    XLOG_ERROR("Failed to initialize robot base");
    return false;
  }

  // create a control context
  ControlContext context;
  context.config = config_;
  context.robot_base = sbot_;
  context.fsm_js_axis_move_queue =
      std::make_shared<ThreadSafeQueue<AxisEvent>>();
  context.fsm_js_button_press_queue =
      std::make_shared<ThreadSafeQueue<JsButton>>();

  // initialize fsm
  ManualMode initial_state{context};
  fsm_ =
      std::make_unique<SbotFsm>(std::move(initial_state), std::move(context));

  return true;
}

void SbotSystem::ControlLoop() {
  XLOG_INFO("SbotSystem: entering control loop");
  Timer timer;
  bool first_run = true;
  TimePoint last_time = Clock::now();
  while (keep_control_loop_) {
    timer.reset();

    // calculate time step
    if (first_run) {
      last_time = Clock::now();
      first_run = false;
      continue;
    }
    auto now = Clock::now();
    double dt =
        std::chrono::duration_cast<std::chrono::microseconds>(now - last_time)
            .count() /
        1000000.0f;
    last_time = now;
    //    if ((dt - 0.002) / 0.002 > 0.1) {
    //      XLOG_ERROR("QuadrupedSystem: control loop running at {} ms", dt *
    //      1000);
    //    }

    // update control
    fsm_->Update();
    timer.sleep_until_us(2000);
  }
  XLOG_INFO("SbotSystem: control loop exited");
}

void SbotSystem::OnJsButtonEvent(const JsButton& btn,
                                 const JxButtonEvent& event) {
  //  std::cout << "Button " << (int)(btn) << " "
  //            << (event == JxButtonEvent::kPress ? "pressed" : "released")
  //            << std::endl;
  if (btn == JsButton::kStart && event == JxButtonEvent::kPress) {
    fsm_->GetContext().fsm_js_button_press_queue->Push(btn);
  }
}

void SbotSystem::OnJsAxisEvent(const JsAxis& axis, const float& value) {
  //  std::cout << "Axis " << (int)(axis) << " value: " << value << std::endl;
  if (axis == JsAxis::kX || axis == JsAxis::kY || axis == JsAxis::kRX) {
    AxisEvent event;
    event.axis = axis;
    event.value = value;
    fsm_->GetContext().fsm_js_axis_move_queue->Push(event);
  }
}

void SbotSystem::Run() {
  // start control thread
  control_thread_ = std::thread(&SbotSystem::ControlLoop, this);

  XLOG_INFO("SbotSystem: entering main loop");
  keep_running_ = true;
  while (keep_running_) {
    fsm_->Update();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  XLOG_INFO("SbotSystem: main loop exited");
}

void SbotSystem::Stop() {
  // stop the control loop
  keep_control_loop_ = false;
  if (control_thread_.joinable()) control_thread_.join();

  // stop the hid input
  hid_event_listener_.reset();
  joystick_->Close();

  // stop the robot
  sbot_->SetSteeringCommand({0, 0, 0, 0});
  sbot_->SetDrivingCommand({0, 0, 0, 0});

  // finally stop the main loop
  keep_running_ = false;
}
}  // namespace xmotion