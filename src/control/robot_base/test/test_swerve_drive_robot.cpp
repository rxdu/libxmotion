/*
 * @file test_swerve_drive_robot.cpp
 * @date 11/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <csignal>
#include <thread>
#include <chrono>
#include <iostream>

#include "motor_waveshare/ddsm_210_array.hpp"
#include "motor_waveshare/sms_sts_servo_array.hpp"
#include "robot_base/swerve_drive_robot.hpp"

using namespace xmotion;

bool keep_running = true;

int main(int argc, char* argv[]) {
  // install signal handler
  std::signal(SIGINT, [](int signum) {
    std::cout << "Exiting..." << std::endl;
    keep_running = false;
  });

  // set up steering motors
  auto steering_motor = std::make_shared<SmsStsServoArray>("/dev/ttyUSB0");
  steering_motor->SetPositionOffset(180);
  steering_motor->SetDefaultPosition(0);
  steering_motor->RegisterMotor(1);
  steering_motor->RegisterMotor(2);
  steering_motor->RegisterMotor(3);
  steering_motor->RegisterMotor(4);
  if (!steering_motor->Connect()) {
    std::cout << "Failed to connect to steering motor" << std::endl;
    return -1;
  }
  //  steering_motor->SetPositions({45, 45, 45, 45});

  // set up driving motors
  auto driving_motor = std::make_shared<Ddsm210Array>("/dev/ttyUSB1");
  driving_motor->RegisterMotor(1);
  driving_motor->RegisterMotor(2);
  driving_motor->RegisterMotor(3);
  driving_motor->RegisterMotor(4);
  if (!driving_motor->Connect()) {
    std::cout << "Failed to connect to driving motor" << std::endl;
    return -1;
  }
  //  driving_motor->SetSpeeds({{1, 10}, {2, 0}, {3, 0}, {4, 0}});

  // set up swerve drive robot
  SwerveDriveRobot::Config config;
  config.kinematics_param.track_width = 0.198;
  config.kinematics_param.wheel_base = 0.245;
  config.kinematics_param.wheel_radius = 0.0363;
  config.steering_motors = steering_motor;
  config.driving_motors = driving_motor;
  config.reverse_right_wheels = true;

  auto robot = std::make_unique<SwerveDriveRobot>(config);

  float factor = 1.0;
  while (keep_running) {
    robot->SetDrivingCommand({0.2, 0.2, 0.2, 0.2});
    float angle = factor * 45 / 180.0 * M_PI;
    robot->SetSteeringCommand({angle, angle, angle, angle});
    std::this_thread::sleep_for(std::chrono::seconds(2));
    factor *= -1;
  }

  return 0;
}