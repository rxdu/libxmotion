/*
 * @file simple_estimator.cpp
 * @date 7/17/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/estimator/simple_estimator.hpp"

namespace xmotion {
void SimpleEstimator::Initialize(const SimpleEstimator::Params& params) {
  params_ = params;
  x_hat_ = params_.x0;
  P_ = params_.p0;
  R_ = params_.r0;
}

void SimpleEstimator::Update() {}
}  // namespace xmotion