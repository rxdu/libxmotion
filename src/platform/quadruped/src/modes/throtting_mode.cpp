/*
 * throtting_mode.cpp
 *
 * Created on 7/4/24 11:31 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/modes/throtting_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
void ThrottingMode::Update(ControlContext& context) {
  std::cout << "ThrottingMode::Update" << std::endl;
}
}  // namespace xmotion