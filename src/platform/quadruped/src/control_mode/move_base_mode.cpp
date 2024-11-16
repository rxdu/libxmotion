/*
 * move_base_mode.cpp
 *
 * Created on 7/4/24 11:30 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/move_base_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
void MoveBaseMode::Update(ControlContext& context) {
  std::cout << "MoveBaseMode::Update" << std::endl;
}
}  // namespace xmotion