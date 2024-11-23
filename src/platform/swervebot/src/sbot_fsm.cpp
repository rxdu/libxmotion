/*
 * @file sbot_fsm.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/sbot_fsm.hpp"

namespace xmotion {
OptionalStateVariant ModeTransition::Transit(ManualMode& state,
                                             ControlContext& context) {
  JsButton btn;
  if (context.js_button_queue->TryPop(btn)) {
    return AutoMode{context};
  }
  SbusMessage sbus;
  if (context.sbus_rc_queue->TryPop(sbus)) {
    if (sbus.channels[6] > 1000) {
      return AutoMode{context};
    }
  }
  //  return ManualMode{context};
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(AutoMode& state,
                                             ControlContext& context) {
  JsButton btn;
  if (context.js_button_queue->TryPop(btn)) {
    return ManualMode{context};
  }
  SbusMessage sbus;
  if (context.sbus_rc_queue->TryPop(sbus)) {
    if (sbus.channels[6] < 1000) {
      return ManualMode{context};
    }
  }
  return std::nullopt;
}
}  // namespace xmotion