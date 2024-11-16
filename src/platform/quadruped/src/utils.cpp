/*
 * utils.cpp
 *
 * Created on 7/13/24 10:43 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/utils.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
double Utils::UpdateRangeLimitedValue(double target, double delta, double min,
                                      double max) {
  double new_value = target + delta;
  if (new_value >= min && new_value <= max) {
    return new_value;
  }
  return target;
}

double Utils::UpdateChangeLimitedValue(double target, double initial,
                                       double delta, double min_change,
                                       double max_change) {
  double change = target + delta - initial;
  if (change >= min_change && change <= max_change) {
    return target + delta;
  }
  return target;
}

double Utils::DegreeToRadian(double degree) { return degree * M_PI / 180.0; }

double Utils::RadianToDegree(double radian) { return radian * 180.0 / M_PI; }

std::optional<HidSettings::KeyFunction> Utils::PollKeyFunction(
    ControlContext &context, HidEventHandler::KeyboardEventType type) {
  auto config = context.system_config;
  auto listener = context.hid_event_listener;

  // return directly if no key event is available
  auto key_event = listener->TryPopKeyboardEvent(type);
  if (!key_event.has_value()) return std::nullopt;

  // keep polling until the latest key event is obtained
  auto next_key_event = listener->TryPopKeyboardEvent(type);
  while (next_key_event.has_value()) {
    key_event = next_key_event;
  }
  XLOG_DEBUG("Keycode: {}", static_cast<int>(key_event->GetKeyCode()));

  return config.hid_settings.keyboard
      .keyboard_mappings[key_event->GetKeyCode()];
}
}  // namespace xmotion