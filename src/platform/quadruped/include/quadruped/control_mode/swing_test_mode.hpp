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
 public:
  SwingTestMode(const ControlContext &context);
  ~SwingTestMode() = default;

  void Update(ControlContext &context) override;

 private:
  void HandleKeyboardInput(ControlContext &context);

  QuadrupedModel::State target_state_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_SWING_TEST_MODE_HPP
