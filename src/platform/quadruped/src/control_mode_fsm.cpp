/*
 * control_mode_fsm.cpp
 *
 * Created on 7/3/24 11:28 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode_fsm.hpp"

#include "quadruped/utils.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
OptionalStateVariant ModeTransition::Transit(FixedStandMode &state,
                                             ControlContext &context) {
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kModeSwitch);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kFixedStandMode) {
      return LyingDownMode{context};
    } else if (key_func.value() == HidSettings::KeyFunction::kSwingTestMode) {
      return SwingTestMode{context};
    } else if (key_func.value() == HidSettings::KeyFunction::kFreeStandMode) {
      return FreeStandMode{context};
    } else if (key_func.value() == HidSettings::KeyFunction::kBalanceTestMode) {
      return BalanceTestMode{context};
    }
  }
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(LyingDownMode &state,
                                             ControlContext &context) {
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kModeSwitch);
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
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kModeSwitch);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kFixedStandMode) {
      return FixedStandMode{context};
    }
  }
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(FreeStandMode &state,
                                             ControlContext &context) {
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kModeSwitch);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kFixedStandMode) {
      return FixedStandMode{context};
    }
  }
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(PassiveMode &state,
                                             ControlContext &context) {
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kModeSwitch);
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

OptionalStateVariant ModeTransition::Transit(BalanceTestMode &state,
                                             ControlContext &context) {
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kModeSwitch);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kFixedStandMode) {
      return FixedStandMode{context};
    }
  }
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(MoveBaseMode &state,
                                             ControlContext &context) {
  return PassiveMode{context};
  // return std::nullopt;
}
}  // namespace xmotion