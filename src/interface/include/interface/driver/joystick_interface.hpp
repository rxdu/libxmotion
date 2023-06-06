/*
 * joystick_interface.hpp
 *
 * Created on 5/31/23 11:10 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_JOYSTICK_INTERFACE_HPP
#define ROBOSW_JOYSTICK_INTERFACE_HPP

#include <string>
#include <functional>

namespace xmotion {
enum class JsButton {
  kTrigger = 0,
  kThumb = 1,
  kThumb2 = 2,
  kTop1 = 3,
  kTop2 = 4,
  kPinkie = 5,
  kBase = 6,
  kBase2 = 7,
  kBase3 = 8,
  kBase4 = 9,
  kBase5 = 10,
  kBase6 = 11,
  kDead = 15,
  kSouth = 16,
  kEast = 17,
  kC = 18,
  kNorth = 19,
  kWest = 20,
  kZ = 21,
  kTL = 22,
  kTR = 23,
  kTL2 = 24,
  kTR2 = 25,
  kSelect = 26,
  kStart = 27,
  kMode = 28,
  kThumbL = 29,
  kThumbR = 30,
};

enum class JsAxis {
  kX = 0,
  kY = 1,
  kZ = 2,
  kRX = 3,
  kRY = 4,
  kRZ = 5,
  kThrottle = 6,
  kRudder = 7,
  kWheel = 8,
  kGas = 9,
  kBrake = 10,
  kHat0X = 16,
  kHat0Y = 17,
  kHat1X = 18,
  kHat1Y = 19,
  kHat2X = 20,
  kHat2Y = 21,
  kHat3X = 22,
  kHat3Y = 23,
  kPressure = 24,
  kDistance = 25,
  kTiltX = 26,
  kTiltY = 27,
  kToolWidth = 28,
};

struct JsAxisValue {
  int min;
  int max;
  float value;
};

struct JoystickDescriptor {
  int index;
  std::string name;
};

class JoystickInterface {
 public:
  using ButtonEventCallback =
      std::function<void(const JsButton& btn, const bool value)>;
  using AxisEventCallback =
      std::function<void(const JsAxis& axis, const JsAxisValue& value)>;

 public:
  virtual ~JoystickInterface() = default;

  // Public API
  virtual bool Open() = 0;
  virtual void Close() = 0;
  virtual bool IsOpened() const = 0;

  virtual std::string GetDeviceName() const = 0;
  virtual std::string GetButtonName(const JsButton& btn) const { return ""; }
  virtual std::string GetAxisName(const JsAxis& axis) const { return ""; }

  virtual bool GetButtonState(const JsButton& btn) const = 0;
  virtual JsAxisValue GetAxisState(const JsAxis& axis) const = 0;

  virtual void SetJoystickRumble(short weakRumble, short strongRumble) {}
};
}  // namespace xmotion

#endif  // ROBOSW_JOYSTICK_INTERFACE_HPP
