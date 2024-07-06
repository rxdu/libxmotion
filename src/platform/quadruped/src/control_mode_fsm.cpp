/*
 * control_mode_fsm.cpp
 *
 * Created on 7/3/24 11:28 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode_fsm.hpp"

namespace xmotion {
OptionalStateVariant ModeTransition::Transit(FixedStandMode &state,
                                             ControlContext &context) {
  return FreeStandMode{};
  // return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(FreeStandMode &state,
                                             ControlContext &context) {
  return MoveBaseMode{};
  // return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(MoveBaseMode &state,
                                             ControlContext &context) {
  return PassiveMode{};
  // return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(PassiveMode &state,
                                             ControlContext &context) {
  //  return ThrottingMode{};
  return state;
  // return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(ThrottingMode &state,
                                             ControlContext &context) {
  return FixedStandMode{};
  // return std::nullopt;
}
}  // namespace xmotion