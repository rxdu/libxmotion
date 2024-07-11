/*
 * quadruped_model.hpp
 *
 * Created on 7/6/24 11:03 PM
 * Description:
 *  A generic class for quadruped robot model with 12DOF (3DOF per leg) with
 *  a similar configuration as the MIT/Unitree robot dog.
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_QUADRUPED_MODEL_HPP
#define QUADRUPED_MOTION_QUADRUPED_MODEL_HPP

#include "interface/type/geometry_types.hpp"

namespace xmotion {
enum class LegIndex : int {
  kFrontRight = 0,
  kFrontLeft = 1,
  kRearRight = 2,
  kRearLeft = 3
};

class QuadrupedModel {
 public:
  using JointVar = Eigen::Matrix<double, 12, 1>;

  struct JointGains {
    JointVar kp;
    JointVar kd;
  };

  struct State {
    JointVar q;
    JointVar q_dot;
    JointVar tau;

    JointVar q_ddot;
  };

 public:
  virtual void SetJointGains(const JointGains& gains) = 0;
  virtual void SetTargetState(const State& state) = 0;
  virtual State GetEstimatedState() = 0;
  virtual void SendCommandToRobot() = 0;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_QUADRUPED_MODEL_HPP
