/*
 * keyboard_interface.hpp
 *
 * Created on 7/5/24 10:18 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_KEYBOARD_INTERFACE_HPP
#define QUADRUPED_MOTION_KEYBOARD_INTERFACE_HPP

#include <string>
#include <functional>

namespace xmotion {
enum class KeyboardCode {
  kUnknown = 0,
  kA,
  kB,
  kC,
  kD,
  kE,
  kF,
  kG,
  kH,
  kI,
  kJ,
  kK,
  kL,
  kM,
  kN,
  kO,
  kP,
  kQ,
  kR,
  kS,
  kT,
  kU,
  kV,
  kW,
  kX,
  kY,
  kZ,
  kEsc,
  kNum1,
  kNum2,
  kNum3,
  kNum4,
  kNum5,
  kNum6,
  kNum7,
  kNum8,
  kNum9,
  kNum0,
  kMinus,
  kEqual,
  kBackspace,
  kTab,
  kEnter,
  kSpace,
  kCapsLock,
  kF1,
  kF2,
  kF3,
  kF4,
  kF5,
  kF6,
  kF7,
  kF8,
  kF9,
  kF10,
  kF11,
  kF12,
  kLeftCtrl,
  kRightCtrl,
  kLeftShift,
  kRightShift,
  kLeftAlt,
  kRightAlt,
  kLeft,
  kRight,
  kUp,
  kDown,
  kHome,
  kEnd,
  kPageUp,
  kPageDown
};

enum class KeyboardEvent {
  kPress,
  kRelease,
};

class KeyboardInterface {
 public:
  using KeyEventCallback = std::function<void(KeyboardCode, KeyboardEvent)>;

 public:
  virtual ~KeyboardInterface() = default;

  virtual bool StartMonitoring(const std::string& event_name) = 0;
  virtual void RegisterKeyEventCallback(KeyEventCallback callback) = 0;

  virtual void PollEvent() = 0;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_KEYBOARD_INTERFACE_HPP
