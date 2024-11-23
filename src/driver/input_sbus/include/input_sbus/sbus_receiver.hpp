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

#include "sbus_decoder.hpp"

namespace xmotion {
class SbusReceiver {
 public:
  SbusReceiver(const std::string& port);
  ~SbusReceiver();

  // public methods
  bool Open();
  void Close();

  using OnSbusMessageReceivedCallback = std::function<void(const SbusMessage&)>;
  void SetOnSbusMessageReceivedCallback(OnSbusMessageReceivedCallback cb);

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};
}  // namespace xmotion

#endif  // SBUS_RECEIVER_HPP