/*
 * hid_handler_interface.hpp
 *
 * Created on 7/6/24 12:15 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_HID_HANDLER_INTERFACE_HPP
#define QUADRUPED_MOTION_HID_HANDLER_INTERFACE_HPP

namespace xmotion {
class HidInputInterface {
 public:
  virtual ~HidInputInterface() = default;

  virtual bool Open() = 0;
  virtual void Close() = 0;
  virtual bool IsOpened() const = 0;

  virtual int GetFd() const = 0;

  virtual void OnInputEvent() = 0;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_HID_HANDLER_INTERFACE_HPP
