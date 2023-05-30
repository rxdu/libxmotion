/*
 * ImuHipnuc.hpp
 *
 * Created on: Nov 22, 2021 22:11
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef IMU_HIPNUC_HPP
#define IMU_HIPNUC_HPP

#include "async_port/async_serial.hpp"
#include "interface/driver/imu_interface.hpp"

namespace robosw {
class ImuHipnuc : public ImuInterface {
 public:
  ImuHipnuc() = default;

  // public API
  bool Connect(std::string uart_name, uint32_t baud_rate = 115200) override;
  void Disconnect() override;
  bool IsConnected() override;

  void GetLastImuData(ImuData *data) override;

 private:
  std::shared_ptr<AsyncSerial> serial_;

  bool Connect(std::string dev_name) { return false; };
  void ParseSerialData(uint8_t *data, const size_t bufsize, size_t len);
};
}  // namespace robosw

#endif /* IMU_HIPNUC_HPP */
