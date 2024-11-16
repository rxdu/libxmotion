/*
 * serial_interface.hpp
 *
 * Created on 5/30/23 10:35 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SERIAL_INTERFACE_HPP
#define XMOTION_SERIAL_INTERFACE_HPP

#include <cstdint>
#include <string>
#include <functional>

namespace xmotion {
class SerialInterface {
 public:
  using ReceiveCallback =
      std::function<void(uint8_t *data, const size_t bufsize, size_t len)>;

 public:
  virtual ~SerialInterface() = default;

  virtual void SetBaudRate(unsigned baudrate) = 0;
  virtual void SetHardwareFlowControl(bool enabled) = 0;

  virtual bool Open() = 0;
  virtual void Close() = 0;
  virtual bool IsOpened() const = 0;

  virtual void SetReceiveCallback(ReceiveCallback cb) = 0;
  virtual void SendBytes(const uint8_t *bytes, size_t length) = 0;
};
}  // namespace xmotion

#endif  // XMOTION_SERIAL_INTERFACE_HPP
