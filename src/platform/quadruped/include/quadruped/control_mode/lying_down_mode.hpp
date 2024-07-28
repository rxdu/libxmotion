/*
 * @file lying_down_mode.hpp
 * @date 7/11/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_LYING_DOWN_MODE_HPP
#define QUADRUPED_MOTION_LYING_DOWN_MODE_HPP

#include "fsm/fsm_template.hpp"
#include "quadruped/control_context.hpp"
#include "time/stopwatch.hpp"

namespace xmotion {
class LyingDownMode : public FsmState<ControlContext> {
 public:
  LyingDownMode(const ControlContext& context);
  void Update(ControlContext& context);

 private:
  StopWatch sw_;
  QuadrupedModel::JointState initial_state_;
  QuadrupedModel::JointState target_state_;
  QuadrupedModel::Command joint_cmd_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_LYING_DOWN_MODE_HPP
