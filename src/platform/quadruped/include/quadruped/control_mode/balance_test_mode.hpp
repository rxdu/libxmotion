/*
 * @file balance_test_mode.hpp
 * @date 7/24/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_BALANCE_TEST_MODE_HPP
#define QUADRUPED_MOTION_BALANCE_TEST_MODE_HPP

#include "fsm/fsm_template.hpp"
#include "quadruped/control_context.hpp"
#include "time/stopwatch.hpp"

namespace xmotion {
class BalanceTestMode : public FsmState<ControlContext> {
  enum class Term { kX, kY, kZ, kYaw };

  struct Pose {
    double x;
    double y;
    double z;
    double yaw;
    Quaterniond quat;
  };

 public:
  BalanceTestMode(const ControlContext& context);
  void Update(ControlContext& context);

 private:
  void UpdateTargetPose(Term term, double delta);
  void HandleKeyboardInput(ControlContext& context);

  ControlSettings::BalanceTestModeParams::PoseLimit pose_limit_;
  ControlSettings::BalanceTestModeParams::PositionController pos_ctrl_gains_;
  ControlSettings::BalanceTestModeParams::OrientationController ori_ctrl_gains_;

  Position3d p0_;
  Quaterniond quat0_;

  Pose target_pose_;
  double move_step_;
  double rotate_step_;

  QuadrupedModel::AllJointGains desired_gains_;
  QuadrupedModel::JointState initial_state_;
  QuadrupedModel::JointState target_state_;
  QuadrupedModel::Command joint_cmd_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_BALANCE_TEST_MODE_HPP