/*
 * app_mekf_estimator.cpp
 *
 * Created on 3/30/24 11:12 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "sensor_imu/imu_hipnuc.hpp"
#include "mekf_ros_node.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto imu = std::make_shared<ImuHipnuc>();
  auto mekf_node = std::make_shared<MekfRosNode>(imu);

  if (!mekf_node->Initialize("/dev/ttyUSB0", 921600)) {
    std::cout << "Failed to initialize MEKF node" << std::endl;
    return -1;
  }

  rclcpp::spin(mekf_node);
  rclcpp::shutdown();

  return 0;
}