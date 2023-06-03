/*
 * sensor_interface.hpp
 *
 * Created on: Dec 15, 2021 10:53
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef SENSOR_INTERFACE_HPP
#define SENSOR_INTERFACE_HPP

#include <string>
#include <stdexcept>
#include <functional>

namespace xmotion {
template <typename T>
class SensorInterface {
 public:
  virtual ~SensorInterface() = default;

  virtual bool Connect(std::string dev_name) {
    throw std::runtime_error(
        "The CAN interface is not implemented by the sensor. You also need to "
        "specify the baudrate if UART interface is intended to be used.");
    return false;
  };

  virtual bool Connect(std::string dev_name, uint32_t baud_rate) {
    throw std::runtime_error(
        "The UART interface is not implemented by the sensor.");
    return false;
  };

  virtual void Disconnect(){};

  virtual bool IsOkay() = 0;

  using CallbackFunc = std::function<void(const T&)>;
  virtual void SetDataReceivedCallback(CallbackFunc cb) = 0;

  virtual T GetLastData() const = 0;
  virtual void GetLastData(T* data) const = 0;

 protected:
  CallbackFunc callback_ = nullptr;
};
}  // namespace xmotion

#endif /* SENSOR_INTERFACE_HPP */
