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