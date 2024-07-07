/*
 * @file unitree_motor.hpp
 * @date 7/6/24
 * @brief
 *  tau = tau_ff + kp * (q_d - q) + kd * (dq_d - dq)
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_UNITREE_MOTOR_HPP
#define QUADRUPED_MOTION_UNITREE_MOTOR_HPP

#include <cstdint>

#include <unitree/idl/go2/LowCmd_.hpp>

namespace xmotion {
class UnitreeMotor {
  // constants from unitree_sdk2
  static constexpr double q_idle_target = 2.146e+9f;
  static constexpr double qd_idle_target = 16000.0f;

 public:
  using CmdMsg = unitree_go::msg::dds_::MotorCmd_;

  // FOC mode (working mode) ->0x01
  // Stop mode (standby mode) ->0x00
  enum class Mode { kIdle = 0x00, kFoc = 0x01 };

  struct State {
    float q;
    float dq;
    float tau;

    float kp;
    float kd;
  };

  using Command = State;

 public:
  UnitreeMotor() = default;

  void SetMode(Mode mode);
  void SetGains(float kp, float kd);
  void SetTarget(float q, float dq, float tau);

  CmdMsg GetCommandMsg();

 private:
  Mode mode_ = Mode::kIdle;
  State state_ = {0, 0, 0, 0, 0};
  CmdMsg cmd_msg_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UNITREE_MOTOR_HPP
