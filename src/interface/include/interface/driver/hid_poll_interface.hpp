/*
 * hid_poll_interface.hpp
 *
 * Created on 7/6/24 12:15 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_HID_POLL_INTERFACE_HPP
#define QUADRUPED_MOTION_HID_POLL_INTERFACE_HPP

namespace xmotion {
class HidPollInterface {
 public:
  virtual ~HidPollInterface() = default;

  virtual void PollEvent() = 0;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_HID_POLL_INTERFACE_HPP
