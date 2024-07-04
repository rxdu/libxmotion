/*
 * passive_mode.cpp
 *
 * Created on 7/4/24 11:30 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/modes/passive_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
void PassiveMode::Update(ControlContext& context) {
  std::cout << "PassiveMode::Update" << std::endl;
}
}  // namespace xmotion