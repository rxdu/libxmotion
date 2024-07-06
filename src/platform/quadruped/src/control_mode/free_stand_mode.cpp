/*
 * free_stand_mode.cpp
 *
 * Created on 7/4/24 11:30 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/free_stand_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
void FreeStandMode::Update(ControlContext& context) {
  std::cout << "FreeStandMode::Update" << std::endl;
}
}  // namespace xmotion