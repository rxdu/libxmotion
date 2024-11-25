/*
 * @file auto_mode.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_AUTO_MODE_HPP
#define XMOTION_AUTO_MODE_HPP

#include "fsm/fsm_template.hpp"
#include "swervebot/control_context.hpp"

namespace xmotion {
class AutoMode : public FsmState<ControlContext> {
 public:
  AutoMode(const ControlContext& context);
  void Update(ControlContext& context);

 private:
};
}  // namespace xmotion

#endif  // XMOTION_AUTO_MODE_HPP