/*
 * imu_hipnuc.cpp
 *
 * Created on: Nov 22, 2021 22:22
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "sensor_imu/imu_hipnuc.hpp"
#include "async_port/async_serial.hpp"

extern "C" {
#include "ch_serial.h"
}

namespace xmotion {
namespace {
void UnpackData(raw_t *data, ImuData *data_imu, int num) {
  data_imu->id = data->imu[num].id;

  data_imu->timestamp = data->imu[num].timestamp;

  data_imu->pressure = data->imu[num].pressure;

  data_imu->accel.x = data->imu[num].acc[0];
  data_imu->accel.y = data->imu[num].acc[1];
  data_imu->accel.z = data->imu[num].acc[2];

  data_imu->gyro.x = data->imu[num].gyr[0];
  data_imu->gyro.y = data->imu[num].gyr[1];
  data_imu->gyro.z = data->imu[num].gyr[2];

  data_imu->magn.x = data->imu[num].mag[0];
  data_imu->magn.y = data->imu[num].mag[1];
  data_imu->magn.z = data->imu[num].mag[2];

  data_imu->euler.roll = data->imu[num].eul[0];
  data_imu->euler.pitch = data->imu[num].eul[1];
  data_imu->euler.yaw = data->imu[num].eul[2];

  data_imu->quat.w = data->imu[num].quat[0];
  data_imu->quat.x = data->imu[num].quat[1];
  data_imu->quat.y = data->imu[num].quat[2];
  data_imu->quat.z = data->imu[num].quat[3];
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
  static raw_t raw;
  for (int i = 0; i < len; i++) {
    int rev = ch_serial_input(&raw, data[i]);

    if (raw.item_code[raw.nitem_code - 1] != KItemGWSOL && rev) {
      ImuData imu;
      UnpackData(&raw, &imu, raw.nimu - 1);

      if (callback_ != nullptr) callback_(imu);

      printf("accel: %6.2f, %6.2f, %6.2f; gyro: %6.2f, %6.2f, %6.2f\n",
             imu.accel.x, imu.accel.y, imu.accel.z, imu.gyro.x, imu.gyro.y,
             imu.gyro.z);
    }
  }
}
}  // namespace xmotion