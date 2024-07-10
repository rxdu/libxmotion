/*
 * control_mode_fsm.cpp
 *
 * Created on 7/3/24 11:28 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode_fsm.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
OptionalStateVariant ModeTransition::Transit(FixedStandMode &state,
                                             ControlContext &context) {
  return FixedStandMode{};
  // return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(FreeStandMode &state,
                                             ControlContext &context) {
  return FreeStandMode{};
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
  auto config = context.system_config;
  auto listener = context.hid_event_listener;
  auto key_event = listener->TryPopKeyboardEvent();
  if (key_event.has_value()) {
    if (config.hid_settings.keyboard.keyboard_mappings[key_event->GetKeyCode()] ==
        HidSettings::KeyFunction::kFixedStandMode) {
      XLOG_INFO("---------> Switch to FixedStandMode");
      return FixedStandMode{};
    }
    XLOG_INFO("Keycode: {}", static_cast<int>(key_event->GetKeyCode()));
  }
  XLOG_INFO("NO TRANSITION");
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(TrottingMode &state,
                                             ControlContext &context) {
  return TrottingMode{};
  // return std::nullopt;
}
}  // namespace xmotion