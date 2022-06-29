/*
 * speed_controller.hpp
 *
 * Created on 4/18/22 11:17 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_SPEED_CONTROLLER_HPP
#define ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_SPEED_CONTROLLER_HPP

#include <string>
#include <thread>
#include <chrono>
#include <memory>
#include <iostream>

#include "interface/types.hpp"
#include "async_port/async_can.hpp"
#include "pid/pid_controller.hpp"

namespace robosw {
class SpeedController {
 public:
  SpeedController() = default;

  void Run(std::string can);
  void Stop();

 private:
  std::string can_dev_;
  std::shared_ptr<AsyncCAN> can_ = nullptr;

  RSTimePoint t0_{RSClock::now()};
  RSTimePoint tl_{RSClock::now()};
  PidController left_pid_controller_{0.2, 0.5, 0, 100, 0.02};
  PidController right_pid_controller_{0.2, 0.5, 0, 100, 0.02};

  void SendPwmCommand(float left, float right);
  void HandleCanFrame(const can_frame *rx_frame);
};
}  // namespace robosw

#endif  // ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_SPEED_CONTROLLER_HPP
