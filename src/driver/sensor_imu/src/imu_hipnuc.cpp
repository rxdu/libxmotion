/*
 * imu_hipnuc.cpp
 *
 * Created on: Nov 22, 2021 22:22
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "sensor_imu/imu_hipnuc.hpp"

#include <cmath>

#include "async_port/async_serial.hpp"

extern "C" {
#include "ch_serial.h"
}

namespace xmotion {
namespace {
static constexpr float g_const = 9.81;

void UnpackData(hipnuc_raw_t *data, ImuData *data_imu) {
  data_imu->id = 0x91;

  data_imu->timestamp = data->imu.ts;

  data_imu->pressure = data->imu.prs;

  data_imu->accel.x = -data->imu.acc[0] * g_const;
  data_imu->accel.y = -data->imu.acc[1] * g_const;
  data_imu->accel.z = -data->imu.acc[2] * g_const;

  data_imu->gyro.x = data->imu.gyr[0] * (M_PI / 180.0);
  data_imu->gyro.y = data->imu.gyr[1] * (M_PI / 180.0);
  data_imu->gyro.z = data->imu.gyr[2] * (M_PI / 180.0);

  data_imu->magn.x = data->imu.mag[0];
  data_imu->magn.y = data->imu.mag[1];
  data_imu->magn.z = data->imu.mag[2];

  data_imu->euler.roll = data->imu.eul[0];
  data_imu->euler.pitch = data->imu.eul[1];
  data_imu->euler.yaw = data->imu.eul[2];

  data_imu->quat.w() = data->imu.quat[0];
  data_imu->quat.x() = data->imu.quat[1];
  data_imu->quat.y() = data->imu.quat[2];
  data_imu->quat.z() = data->imu.quat[3];
}
}  // namespace

bool ImuHipnuc::Connect(std::string uart_name, uint32_t baud_rate) {
  if (!serial_) serial_ = std::make_shared<AsyncSerial>(uart_name, baud_rate);
  if (serial_->IsOpened()) serial_->Close();

  serial_->SetReceiveCallback(
      std::bind(&ImuHipnuc::ParseSerialData, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  return serial_->Open() && serial_->IsOpened();
}

void ImuHipnuc::Disconnect() { serial_->Close(); }

bool ImuHipnuc::IsConnected() { return serial_->IsOpened(); }

void ImuHipnuc::GetLastImuData(ImuData *data) {}

void ImuHipnuc::ParseSerialData(uint8_t *data, const size_t bufsize,
                                size_t len) {
  static hipnuc_raw_t raw;
  for (int i = 0; i < len; i++) {
    int rev = ch_serial_input(&raw, data[i]);

    if (rev) {
      ImuData imu;
      UnpackData(&raw, &imu);

      if (callback_ != nullptr) callback_(imu);

      //      printf("accel: %6.2f, %6.2f, %6.2f; gyro: %6.2f, %6.2f, %6.2f\n",
      //             imu.accel.x, imu.accel.y, imu.accel.z, imu.gyro.x,
      //             imu.gyro.y, imu.gyro.z);
    }
  }
}
}  // namespace xmotion