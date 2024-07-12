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
namespace {
std::optional<HidSettings::KeyFunction> PollKeyFunction(
    ControlContext &context) {
  auto config = context.system_config;
  auto listener = context.hid_event_listener;
  auto key_event = listener->TryPopKeyboardEvent();

  std::optional<HidSettings::KeyFunction> result = std::nullopt;
  if (key_event.has_value()) {
    result =
        config.hid_settings.keyboard.keyboard_mappings[key_event->GetKeyCode()];
    XLOG_DEBUG("Keycode: {}", static_cast<int>(key_event->GetKeyCode()));
  }
  return result;
}
}  // namespace

OptionalStateVariant ModeTransition::Transit(FixedStandMode &state,
                                             ControlContext &context) {
  auto key_func = PollKeyFunction(context);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kFixedStandMode) {
      return LyingDownMode{context};
    }
  }
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(LyingDownMode &state,
                                             ControlContext &context) {
  auto key_func = PollKeyFunction(context);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kPassiveMode) {
      return PassiveMode{context};
    } else if (key_func.value() == HidSettings::KeyFunction::kFixedStandMode) {
      return FixedStandMode{context};
    }
  }
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(SwingTestMode &state,
                                             ControlContext &context) {
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(FreeStandMode &state,
                                             ControlContext &context) {
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(MoveBaseMode &state,
                                             ControlContext &context) {
  return PassiveMode{context};
  // return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(PassiveMode &state,
                                             ControlContext &context) {
  auto key_func = PollKeyFunction(context);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kFixedStandMode) {
      return FixedStandMode{context};
    }
  }
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(TrottingMode &state,
                                             ControlContext &context) {
  return TrottingMode{};
  // return std::nullopt;
}
}  // namespace xmotion