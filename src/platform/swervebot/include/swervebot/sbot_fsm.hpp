/*
 * @file sbot_fsm.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SBOT_FSM_HPP
#define XMOTION_SBOT_FSM_HPP

#include "fsm/fsm_template.hpp"

#include "swervebot/control_context.hpp"

#include "swervebot/control_modes/manual_mode.hpp"
#include "swervebot/control_modes/auto_mode.hpp"

namespace xmotion {
using StateVariant = std::variant<ManualMode, AutoMode>;
using OptionalStateVariant = std::optional<StateVariant>;

struct ModeTransition {
  static OptionalStateVariant Transit(ManualMode &state,
                                      ControlContext &context);
  static OptionalStateVariant Transit(AutoMode &state, ControlContext &context);
};

using SbotFsm =
    FiniteStateMachine<ControlContext, ModeTransition, ManualMode, AutoMode>;
}  // namespace xmotion

#endif  // XMOTION_SBOT_FSM_HPP