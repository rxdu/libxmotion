/*
 * quadruped_model.hpp
 *
 * Created on 7/6/24 11:03 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_QUADRUPED_MODEL_HPP
#define QUADRUPED_MOTION_QUADRUPED_MODEL_HPP

namespace xmotion {
enum class LegIndex : int {
  kFrontRight = 0,
  kFrontLeft = 1,
  kRearRight = 2,
  kRearLeft = 3
};

class QuadrupedModel {
 public:
  virtual void SendCommandToRobot() = 0;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_QUADRUPED_MODEL_HPP
