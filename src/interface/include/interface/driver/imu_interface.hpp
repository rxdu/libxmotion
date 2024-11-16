/*
 * imu.hpp
 *
 * Created on: Nov 22, 2021 22:10
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef XMOTION_IMU_INTERFACE_HPP
#define XMOTION_IMU_INTERFACE_HPP

#include <string>

#include "interface/types.hpp"

namespace xmotion {
struct ImuData {
  uint32_t id;
  Vector3f accel;
  Vector3f gyro;
  Vector3f magn;
  Euler euler;
  Quaterniond quat;
  float pressure;
  uint32_t timestamp;
};

class ImuInterface {
 public:
  virtual ~ImuInterface() = default;

  virtual bool Connect(std::string dev_name) = 0;
  virtual bool Connect(std::string dev_name, uint32_t baud_rate) = 0;

  virtual void Disconnect() = 0;
  virtual bool IsConnected() = 0;

  using ImuCallbackFunc = std::function<void(const ImuData &data)>;
  virtual void SetCallback(ImuCallbackFunc cb) { callback_ = cb; }

  virtual void GetLastImuData(ImuData *data) = 0;

 protected:
  ImuCallbackFunc callback_ = nullptr;
};
}  // namespace xmotion

#endif /* XMOTION_IMU_INTERFACE_HPP */
