/*
 * mekf_ros_node.cpp
 *
 * Created on 3/31/24 7:51 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "mekf_ros_node.hpp"

#include <thread>
#include <chrono>
#include <stdexcept>

namespace xmotion {
MekfRosNode::MekfRosNode(std::shared_ptr<ImuInterface> imu)
    : rclcpp::Node("mekf_ros_node"), imu_(imu) {
  last_update_time_ = Clock::now();
  if (imu_ != nullptr) {
    imu_->SetCallback(
        std::bind(&MekfRosNode::ImuCallback, this, std::placeholders::_1));
  } else {
    throw std::runtime_error("IMU interface is nullptr");
  }

  // setup ros
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
}

bool MekfRosNode::Initialize(std::string dev_name, uint32_t baud_rate) {
  if (baud_rate == 0) {
    if (!imu_->Connect(dev_name)) {
      std::cout << "Failed to open device" << std::endl;
      return false;
    }
  } else {
    if (!imu_->Connect(dev_name, baud_rate)) {
      std::cout << "Failed to open device" << std::endl;
      return false;
    }
  }

  return true;
}

void MekfRosNode::ImuCallback(const ImuData& data) {
  auto error = Clock::now() - last_update_time_;
  last_update_time_ = Clock::now();

  // publish imu data
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = this->now();
  imu_msg.header.frame_id = "imu_link";
  imu_msg.angular_velocity.x = data.gyro.x;
  imu_msg.angular_velocity.y = data.gyro.y;
  imu_msg.angular_velocity.z = data.gyro.z;
  imu_msg.linear_acceleration.x = data.accel.x;
  imu_msg.linear_acceleration.y = data.accel.y;
  imu_msg.linear_acceleration.z = data.accel.z;
  imu_pub_->publish(imu_msg);

  auto time_lapse =
      std::chrono::duration_cast<std::chrono::microseconds>(error).count();

  RCLCPP_INFO(this->get_logger(), "imu update rate: %f",
              1.0f / time_lapse * 1000000.0f);
}
}  // namespace xmotion