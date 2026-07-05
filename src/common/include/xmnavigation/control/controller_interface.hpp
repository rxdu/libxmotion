/*
 * controller_interface.hpp
 *
 * Created 4/15/22 5:02 PM
 * Description:
 * 
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#pragma once

namespace xmotion {
template<typename State, typename Output>
class ControllerInterface {
 public:
  virtual ~ControllerInterface() = default;

  virtual Output Update(const State &target, const State &estimated) = 0;
};
}  // namespace xmotion

