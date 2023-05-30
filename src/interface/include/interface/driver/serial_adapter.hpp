/*
 * serial_adapter.hpp
 *
 * Created on 5/30/23 10:35 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SERIAL_ADAPTER_HPP
#define ROBOSW_SERIAL_ADAPTER_HPP

#include <cstdint>
#include <string>
#include <functional>

namespace robosw {
class SerialAdapter {
 public:
  using ReceiveCallback =
      std::function<void(uint8_t *data, const size_t bufsize, size_t len)>;

 public:
  virtual ~SerialAdapter() = default;

  virtual void SetBaudRate(unsigned baudrate) = 0;
  virtual void SetHardwareFlowControl(bool enabled) = 0;

  virtual bool Open() = 0;
  virtual void Close() = 0;
  virtual bool IsOpened() const = 0;

  virtual void SetReceiveCallback(ReceiveCallback cb) = 0;
  virtual void SendBytes(const uint8_t *bytes, size_t length) = 0;
};
}  // namespace robosw

#endif  // ROBOSW_SERIAL_ADAPTER_HPP
