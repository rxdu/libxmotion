/*
 * controller_interface.hpp
 *
 * Created 4/15/22 5:02 PM
 * Description:
 * 
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_INTERFACE_INCLUDE_INTERFACE_CONTROLLER_INTERFACE_HPP
#define ROBOSW_SRC_INTERFACE_INCLUDE_INTERFACE_CONTROLLER_INTERFACE_HPP

namespace xmotion {
template<typename State, typename Output>
class ControllerInterface {
 public:
  virtual Output Update(const State &target, const State &estimated) = 0;
};
}

#endif //ROBOSW_SRC_INTERFACE_INCLUDE_INTERFACE_CONTROLLER_INTERFACE_HPP
