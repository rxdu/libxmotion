/*
 * @file balance_test_mode.cpp
 * @date 7/24/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/balance_test_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
BalanceTestMode::BalanceTestMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to BalanceTestMode");
}

void BalanceTestMode::Update(ControlContext& context) {
  // finally send the command to the robot
  context.robot_model->SendCommandToRobot();
}
}  // namespace xmotion