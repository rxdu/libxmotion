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

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "imu_markers", 10);
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

  // initialize MEKF
  params_.gravity_constant = 9.81;
  params_.init_quaternion = Eigen::Quaterniond(1, 0, 0, 0);
  params_.init_state = Eigen::VectorXd::Zero(Mekf6::StateDimension);
  params_.init_state(0) = 0;
  params_.init_state(1) = 0;
  params_.init_state(2) = 0;
  params_.init_state_cov =
      1.0 *
      Eigen::MatrixXd::Identity(Mekf6::StateDimension, Mekf6::StateDimension);
  params_.init_observation_noise_cov =
      1.0 * Eigen::MatrixXd::Identity(Mekf6::ObservationDimension,
                                      Mekf6::ObservationDimension);
  params_.sigma_omega = Eigen::Vector3d(0.01, 0.01, 0.01);
  params_.sigma_f = Eigen::Vector3d(0.01, 0.01, 0.01);
  params_.sigma_beta_omega = Eigen::Vector3d(0.01, 0.01, 0.01);
  params_.sigma_beta_f = Eigen::Vector3d(0.01, 0.01, 0.01);

  mekf6_.Initialize(params_);

  return true;
}

void MekfRosNode::ImuCallback(const ImuData& data) {
  auto error = Clock::now() - last_update_time_;
  last_update_time_ = Clock::now();
  auto time_lapse =
      std::chrono::duration_cast<std::chrono::microseconds>(error).count();
  
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

  // update mekf
  Mekf6::ControlInput gyro_tilde;
  gyro_tilde << data.gyro.x, data.gyro.y, data.gyro.z;
  Mekf6::Observation accel_tilde;
  accel_tilde << data.accel.x, data.accel.y, data.accel.z;
  mekf6_.Update(gyro_tilde, accel_tilde, 1.0 / 500.0);

  // publish a marker
  visualization_msgs::msg::MarkerArray marker_array;

  // Reference frame marker
  visualization_msgs::msg::Marker reference_frame;
  reference_frame.header.frame_id = "world";
  reference_frame.header.stamp = this->now();
  reference_frame.ns = "imu_quaternion";
  reference_frame.id = 0;
  reference_frame.type = visualization_msgs::msg::Marker::CUBE;
  reference_frame.action = visualization_msgs::msg::Marker::ADD;
  reference_frame.pose.position.x = 0.0;
  reference_frame.pose.position.y = 0.0;
  reference_frame.pose.position.z = 0.0;
  reference_frame.pose.orientation.x = 0.0;
  reference_frame.pose.orientation.y = 0.0;
  reference_frame.pose.orientation.z = 0.0;
  reference_frame.pose.orientation.w = 1.0;
  reference_frame.scale.x = 0.3;
  reference_frame.scale.y = 0.3;
  reference_frame.scale.z = 0.3;
  reference_frame.color.a = 0.5;
  reference_frame.color.r = 0.0;
  reference_frame.color.g = 1.0;
  reference_frame.color.b = 0.0;

  // Orientation marker
  visualization_msgs::msg::Marker orientation_marker;
  orientation_marker.header.frame_id = "world";
  orientation_marker.header.stamp = this->now();
  orientation_marker.ns = "imu_quaternion";
  orientation_marker.id = 1;
  orientation_marker.type = visualization_msgs::msg::Marker::CUBE;
  orientation_marker.action = visualization_msgs::msg::Marker::ADD;
  orientation_marker.pose.position.x = 0.0;
  orientation_marker.pose.position.y = 0.0;
  orientation_marker.pose.position.z = 0.0;
  orientation_marker.pose.orientation.x = mekf6_.GetQuaternion().x();
  orientation_marker.pose.orientation.y = mekf6_.GetQuaternion().y();
  orientation_marker.pose.orientation.z = mekf6_.GetQuaternion().z();
  orientation_marker.pose.orientation.w = mekf6_.GetQuaternion().w();
  orientation_marker.scale.x = 0.5;
  orientation_marker.scale.y = 0.5;
  orientation_marker.scale.z = 0.5;
  orientation_marker.color.a = 1.0;
  orientation_marker.color.r = 1.0;
  orientation_marker.color.g = 0.0;
  orientation_marker.color.b = 0.0;

  marker_array.markers.push_back(reference_frame);
  marker_array.markers.push_back(orientation_marker);
  marker_pub_->publish(marker_array);

  RCLCPP_INFO(this->get_logger(),
              "gyro: %.6f %.6f %.6f, accel: %.6f %.6f %.6f, mag: %.6f %.6f "
              "%.6f, rate: %.1f",
              data.gyro.x, data.gyro.y, data.gyro.z, data.accel.x, data.accel.y,
              data.accel.z, data.magn.x, data.magn.y, data.magn.z,
              1.0f / time_lapse * 1000000.0f);

  RCLCPP_INFO(this->get_logger(), "q(w,x,y,z): %.6f, %.6f, %.6f, %.6f",
              mekf6_.GetQuaternion().w(), mekf6_.GetQuaternion().x(),
              mekf6_.GetQuaternion().y(), mekf6_.GetQuaternion().z());

  //  RCLCPP_INFO(this->get_logger(), "imu update rate: %f",
  //              1.0f / time_lapse * 1000000.0f);
}
}  // namespace xmotion