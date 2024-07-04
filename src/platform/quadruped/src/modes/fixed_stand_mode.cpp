/*
 * fixed_stand_mode.cpp
 *
 * Created on 7/4/24 11:30 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/modes/fixed_stand_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
void FixedStandMode::Update(ControlContext& context) {
  std::cout << "FixedStandMode::Update" << std::endl;
}
}  // namespace xmotion