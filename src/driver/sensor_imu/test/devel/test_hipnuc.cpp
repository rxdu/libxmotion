/*
 * test_hipnuc.cpp
 *
 * Created on: Nov 22, 2021 21:47
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "driver/imu_driver/imu_hipnuc.hpp"

using namespace xmotion;

int main(int argc, char **argv) {
  ImuHipnuc imu;

  if (!imu.Connect("/dev/ttyUSB0", 115200)) {
    std::cout << "Failed to open device" << std::endl;
    return -1;
  }

  uint8_t count = 0;
  while (imu.IsConnected()) {
    sleep(1);
  }

  return 0;
}