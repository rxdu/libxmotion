/* 
 * pid_controller.cpp
 *
 * Created on 4/11/22 11:33 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "pid/pid_controller.hpp"

#include <cmath>

namespace robosw {
namespace {
float CalcAlphaEMA(float fn) {
  if (fn <= 0)
    return 1;
  // α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
  const float c = std::cos(2 * float(M_PI) * fn);
  return c - 1 + std::sqrt(c * c - 4 * c + 3);
}
}

PidController::PidController(float kp, float ki, float kd, float fc, float ts)
    : kp_(kp), ki_(ki), kd_(kd), alpha_(CalcAlphaEMA(fc * ts)), ts_(ts) {}

float PidController::Update(float reference, float measurement) {
  float error = reference - measurement;

  float ef = alpha_ * error + (1 - alpha_) * ef_last_;
  float derivative = (ef - ef_last_) / ts_;
  float new_integral = integral_ + error * ts_;

  float u = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // anti-windup
  if (u > u_max_) {
    u = u_max_;
  } else if (u < -u_max_) {
    u = -u_max_;
  } else {
    integral_ = new_integral;
  }

  ef_last_ = ef;

  return u;
}
}