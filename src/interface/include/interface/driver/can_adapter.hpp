/*
 * can_adapter.hpp
 *
 * Created on 5/30/23 10:28 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_CAN_ADAPTER_HPP
#define ROBOSW_CAN_ADAPTER_HPP

#include <linux/can.h>

#include <string>
#include <functional>

namespace xmotion {
class CanAdapter {
 public:
  using ReceiveCallback = std::function<void(const struct can_frame *rx_frame)>;

 public:
  virtual ~CanAdapter() = default;

  // Public API
  virtual bool Open() = 0;
  virtual void Close() = 0;
  virtual bool IsOpened() const = 0;

  virtual void SetReceiveCallback(ReceiveCallback cb) = 0;
  virtual void SendFrame(const struct can_frame &frame) = 0;
};
}  // namespace xmotion

#endif  // ROBOSW_CAN_ADAPTER_HPP
