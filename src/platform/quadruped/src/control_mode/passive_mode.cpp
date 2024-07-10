/*
 * passive_mode.cpp
 *
 * Created on 7/4/24 11:30 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/passive_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
PassiveMode::PassiveMode() {}

void PassiveMode::Update(ControlContext& context) {
  XLOG_INFO("PassiveMode::Update");
  // set joint positions to current joint positions


  XLOG_INFO("PassiveMode: Update end");
}
}  // namespace xmotion