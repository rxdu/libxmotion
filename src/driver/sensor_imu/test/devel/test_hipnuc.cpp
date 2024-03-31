/*
 * test_hipnuc.cpp
 *
 * Created on: Nov 22, 2021 21:47
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>
#include <thread>
#include <chrono>

#include "sensor_imu/imu_hipnuc.hpp"

using namespace xmotion;

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

TimePoint last_update_time;

void ImuCallback(const ImuData &data) {
  auto error = Clock::now() - last_update_time;
  last_update_time = Clock::now();

  std::cout << "Accel: " << data.accel.x << " " << data.accel.y << " "
            << data.accel.z << " Gyro: " << data.gyro.x << " " << data.gyro.y
            << " " << data.gyro.z << ", Update rate: "
            << 1.0f /
                   std::chrono::duration_cast<std::chrono::microseconds>(error)
                       .count() *
                   1000000.0f
            << std::endl;
}

int main(int argc, char **argv) {
  ImuHipnuc imu;

  imu.SetCallback(ImuCallback);
  if (!imu.Connect("/dev/ttyUSB0", 921600)) {
    std::cout << "Failed to open device" << std::endl;
    return -1;
  }

  uint8_t count = 0;
  while (imu.IsConnected()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}