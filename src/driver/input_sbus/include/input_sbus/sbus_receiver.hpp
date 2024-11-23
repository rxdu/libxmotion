/*
 * @file sbus_receiver.hpp
 * @date 11/23/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef SBUS_RECEIVER_HPP
#define SBUS_RECEIVER_HPP

#include <memory>
#include <functional>

#include "interface/driver/rc_receiver_interface.hpp"
#include "input_sbus/sbus_decoder.hpp"

namespace xmotion {
class SbusReceiver : public RcReceiverInterface {
 public:
  SbusReceiver(const std::string& port);
  ~SbusReceiver();

  // public methods
  bool Open() override;
  void Close() override;
  bool IsOpened() const override;

  void SetOnRcMessageReceivedCallback(OnRcMessageReceivedCallback cb) override;

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};
}  // namespace xmotion

#endif  // SBUS_RECEIVER_HPP