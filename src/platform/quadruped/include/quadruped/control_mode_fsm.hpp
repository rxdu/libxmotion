/*
 * @file control_mode_fsm.hpp
 * @date 7/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_CONTROL_MODE_FSM_HPP
#define QUADRUPED_MOTION_CONTROL_MODE_FSM_HPP

#include "fsm/fsm_template.hpp"

#include "quadruped/control_context.hpp"

#include "quadruped/control_mode/fixed_stand_mode.hpp"
#include "quadruped/control_mode/lying_down_mode.hpp"
#include "quadruped/control_mode/swing_test_mode.hpp"
#include "quadruped/control_mode/free_stand_mode.hpp"
#include "quadruped/control_mode/move_base_mode.hpp"
#include "quadruped/control_mode/passive_mode.hpp"
#include "quadruped/control_mode/trotting_mode.hpp"

namespace xmotion {
using StateVariant =
    std::variant<FixedStandMode, LyingDownMode, SwingTestMode, FreeStandMode,
                 MoveBaseMode, PassiveMode, TrottingMode>;
using OptionalStateVariant = std::optional<StateVariant>;

struct ModeTransition {
  static OptionalStateVariant Transit(FixedStandMode &state,
                                      ControlContext &context);

  static OptionalStateVariant Transit(LyingDownMode &state,
                                      ControlContext &context);

  static OptionalStateVariant Transit(SwingTestMode &state,
                                      ControlContext &context);

  static OptionalStateVariant Transit(FreeStandMode &state,
                                      ControlContext &context);

  static OptionalStateVariant Transit(MoveBaseMode &state,
                                      ControlContext &context);

  static OptionalStateVariant Transit(PassiveMode &state,
                                      ControlContext &context);

  static OptionalStateVariant Transit(TrottingMode &state,
                                      ControlContext &context);
};

using ControlModeFsm =
    FiniteStateMachine<ControlContext, ModeTransition, FixedStandMode,
                       LyingDownMode, SwingTestMode, FreeStandMode,
                       MoveBaseMode, PassiveMode, TrottingMode>;
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_CONTROL_MODE_FSM_HPP
