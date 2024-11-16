/*
 * @file ws_sbot_base.hpp
 * @date 11/13/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_WS_SBOT_BASE_HPP
#define XMOTION_WS_SBOT_BASE_HPP

#include "swervebot/sbot_config.hpp"

#include "motor_waveshare/ddsm_210_array.hpp"
#include "motor_waveshare/sms_sts_servo_array.hpp"
#include "robot_base/swerve_drive_robot.hpp"

namespace xmotion {
class WsSbotBase {
  using ModelConfig = SwerveDriveRobot::Config;

 public:
  WsSbotBase(const SbotConfig::BaseSettings &config);

  // public methods
  bool Initialize();

  // low-level commands
  void SetSteeringCommand(const std::array<float, 4> &angles);
  void SetDrivingCommand(const std::array<float, 4> &speeds);

 private:
  static constexpr float sbot_track_width = 0.198;
  static constexpr float sbot_wheel_base = 0.245;
  static constexpr float sbot_wheel_radius = 0.0363;

  SbotConfig::BaseSettings config_;
  std::shared_ptr<SmsStsServoArray> steering_motor_;
  std::shared_ptr<Ddsm210Array> driving_motor_;
  std::unique_ptr<SwerveDriveRobot> robot_;
};
}  // namespace xmotion

#endif  // XMOTION_WS_SBOT_BASE_HPP