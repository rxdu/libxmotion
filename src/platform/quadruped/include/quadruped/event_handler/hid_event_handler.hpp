/*
 * @file hid_event_handler.hpp
 * @date 7/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_HID_EVENT_HANDLER_HPP
#define QUADRUPED_HID_EVENT_HANDLER_HPP

#include "input_hid/keyboard_handler.hpp"
#include "input_hid/hid_event_listener.hpp"

#include "quadruped/system_config.hpp"
#include "quadruped/event_handler/hid_event.hpp"
#include "quadruped/event_handler/event_queue.hpp"

namespace xmotion {
class HidEventHandler {
 public:
  enum class KeyboardEventType {
    kModeSwitch = 0,
    kControlInput,
  };

  explicit HidEventHandler(const HidSettings& config);

  bool Initialize();
  void Start();

  std::optional<HidEvent> TryPopJoystickEvent();
  std::optional<HidEvent> TryPopKeyboardEvent(KeyboardEventType type);

 private:
  void OnKeyEvent(KeyboardCode code, KeyboardEvent event);

  HidSettings config_;
  uint32_t queue_size_;

  KeyboardHandler keyboard_handler_;
  HidEventListener hid_event_listener_;

  EventQueue<HidEvent> js_event_queue_;
  EventQueue<HidEvent> kb_mode_switch_queue_;
  EventQueue<HidEvent> kb_control_input_queue_;
};
}  // namespace xmotion

#endif  // QUADRUPED_HID_EVENT_HANDLER_HPP
