/*
 * @file control_mode_fsm.hpp
 * @date 7/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_CONTROL_MODE_FSM_HPP
#define QUADRUPED_MOTION_CONTROL_MODE_FSM_HPP

#include "quadruped/fsm_template.hpp"

#include "quadruped/control_context.hpp"
#include "quadruped/modes/fixed_stand_mode.hpp"
#include "quadruped/modes/free_stand_mode.hpp"
#include "quadruped/modes/move_base_mode.hpp"
#include "quadruped/modes/passive_mode.hpp"
#include "quadruped/modes/throtting_mode.hpp"

namespace xmotion {
using StateVariant = std::variant<FixedStandMode, FreeStandMode, MoveBaseMode,
                                  PassiveMode, ThrottingMode>;
using OptionalStateVariant = std::optional<StateVariant>;

struct ModeTransition {
  static OptionalStateVariant Transit(FixedStandMode &state,
                                      ControlContext &context) {
    return FreeStandMode{};
    // return std::nullopt;
  }

  static OptionalStateVariant Transit(FreeStandMode &state,
                                      ControlContext &context) {
    return MoveBaseMode{};
    // return std::nullopt;
  }

  static OptionalStateVariant Transit(MoveBaseMode &state,
                                      ControlContext &context) {
    return PassiveMode{};
    // return std::nullopt;
  }

  static OptionalStateVariant Transit(PassiveMode &state,
                                      ControlContext &context) {
    return ThrottingMode{};
    // return std::nullopt;
  }

  static OptionalStateVariant Transit(ThrottingMode &state,
                                      ControlContext &context) {
    return FixedStandMode{};
    // return std::nullopt;
  }
};

using ControlModeFsm =
    FiniteStateMachine<ControlContext, ModeTransition, FixedStandMode,
                       FreeStandMode, MoveBaseMode, PassiveMode, ThrottingMode>;
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_CONTROL_MODE_FSM_HPP
