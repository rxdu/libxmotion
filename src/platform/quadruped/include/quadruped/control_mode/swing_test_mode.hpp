/*
 * @file swing_test_mode.hpp
 * @date 7/12/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_SWING_TEST_MODE_HPP
#define QUADRUPED_MOTION_SWING_TEST_MODE_HPP

#include "fsm/fsm_template.hpp"
#include "quadruped/control_context.hpp"

namespace xmotion {
class SwingTestMode : public FsmState<ControlContext> {
  enum class Axis { kX, kY, kZ };

 public:
  SwingTestMode(const ControlContext &context);
  ~SwingTestMode() = default;

  void Update(ControlContext &context) override;

 private:
  void HandleKeyboardInput(ControlContext &context);
  void UpdateTargetFootPosition(Axis axis, double delta);

  LegIndex swing_leg_;
  double move_step_;
  ControlSettings::SwingTestModeParams::ChangeLimit change_limit_;
  Position3d initial_position_;
  Position3d target_position_;
  Eigen::Matrix<double, 3, 3> kp_;
  Eigen::Matrix<double, 3, 3> kd_;
  QuadrupedModel::Command joint_cmd_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_SWING_TEST_MODE_HPP
