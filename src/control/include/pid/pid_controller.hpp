/* 
 * pid_controller.hpp
 *
 * Created on 4/11/22 11:33 PM
 * Description:
 *
 * Reference:
 * [1] https://tttapa.github.io/Pages/Arduino/Control-Theory/Motor-Fader/PID-Controllers.html
 * [2] https://tttapa.github.io/Pages/Arduino/Control-Theory/Motor-Fader/PID-Cpp-Implementation.html
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_CONTROL_PID_INCLUDE_PID_PID_CONTROLLER_HPP
#define ROBOSW_SRC_CONTROL_PID_INCLUDE_PID_PID_CONTROLLER_HPP

#include <eigen3/Eigen/Core>

namespace robosw {
//template<typename N>
class PidController {
 public:
  PidController(float kp, float ki, float kd, float umax, float ts);

  // public API
  float Update(float reference, float measurement);

 private:
  float kp_, ki_, kd_;
  float ts_;
  float u_max_ = 100;

  float integral_ = 0;
  float error_last_ = 0;
};
}

#endif //ROBOSW_SRC_CONTROL_PID_INCLUDE_PID_PID_CONTROLLER_HPP
