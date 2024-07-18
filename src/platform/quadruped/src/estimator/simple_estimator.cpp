/*
 * @file simple_estimator.cpp
 * @date 7/17/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/estimator/simple_estimator.hpp"

#include "quadruped/robot_model/quadruped_model.hpp"
#include "quadruped/matrix_helper.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
namespace {
/**
 * @brief Calculate foot position wrt body in world frame p_sfB
 * @param robot_model
 * @param q
 * @param orientation
 * @return
 */
std::array<Position3d, 4> GetFootToBodyPositionWrtWorld(
    std::shared_ptr<QuadrupedModel> robot_model,
    const QuadrupedModel::AllJointVar& q, const Quaterniond& orientation) {
  std::array<Position3d, 4> foot_pos;
  for (int i = 0; i < 4; ++i) {
    auto index = static_cast<LegIndex>(i);
    foot_pos[i] = robot_model->GetFootPosition(index, q.segment<3>(i * 3),
                                               QuadrupedModel::RefFrame::kBase);
    foot_pos[i] = orientation.toRotationMatrix() * foot_pos[i];
  }
  return foot_pos;
}

/**
 * @brief Calculate foot velocity wrt body in world frame v_sfB
 * @param robot_model
 * @param q
 * @param q_dot
 * @param orientation
 * @param gyro
 * @return
 */
std::array<Velocity3d, 4> GetFootToBodyVelocityWrtWorld(
    std::shared_ptr<QuadrupedModel> robot_model,
    const QuadrupedModel::AllJointVar& q,
    const QuadrupedModel::AllJointVar& q_dot, const Quaterniond& orientation,
    const Eigen::Vector3d& gyro) {
  std::array<Velocity3d, 4> foot_vel;
  for (int i = 0; i < 4; ++i) {
    auto index = static_cast<LegIndex>(i);
    foot_vel[i] = robot_model->GetFootVelocity(index, q.segment<3>(i * 3),
                                               q_dot.segment<3>(i * 3));
    foot_vel[i] +=
        MatrixHelper::GetSkewSymmetricMatrix(gyro) *
        robot_model->GetFootPosition(index, q.segment<3>(i * 3),
                                     QuadrupedModel::RefFrame::kBase);
    foot_vel[i] = orientation.toRotationMatrix() * foot_vel[i];
  }
  return foot_vel;
}
}  // namespace

void SimpleEstimator::Initialize(const SimpleEstimator::Params& params) {
  params_ = params;
  x_hat_ = params_.x0;
  P_ = params_.p0;
  R_ = params_.r0;
}

void SimpleEstimator::Update() { XLOG_INFO("estimator update"); }
}  // namespace xmotion