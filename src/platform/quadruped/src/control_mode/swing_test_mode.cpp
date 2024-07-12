/*
 * swing_test_mode.cpp
 *
 * Created on 7/12/24 10:16 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/swing_test_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
SwingTestMode::SwingTestMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to SwingTestMode");
}

void SwingTestMode::Update(ControlContext& context) {}
}  // namespace xmotion