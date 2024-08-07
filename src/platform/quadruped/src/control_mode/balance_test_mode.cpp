/*
 * @file balance_test_mode.cpp
 * @date 7/24/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/balance_test_mode.hpp"

#include "quadruped/utils.hpp"
#include "logging/xlogger.hpp"

namespace xmotion {
BalanceTestMode::BalanceTestMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to BalanceTestMode");

  pose_limit_ =
      context.system_config.ctrl_settings.balance_test_mode.pose_limit;
  pos_ctrl_gains_ =
      context.system_config.ctrl_settings.balance_test_mode.position_controller;
  ori_ctrl_gains_ = context.system_config.ctrl_settings.balance_test_mode
                        .orientation_controller;

  move_step_ = context.system_config.ctrl_settings.balance_test_mode.move_step;
  rotate_step_ =
      context.system_config.ctrl_settings.balance_test_mode.rotate_step;

  p0_ = context.estimator->GetEstimatedBasePosition();
  quat0_ = context.estimator->GetEstimatedBaseOrientation();
}

void BalanceTestMode::UpdateTargetPose(BalanceTestMode::Term term,
                                       double delta) {
  if (term == Term::kX) {
    target_pose_.x = Utils::UpdateRangeLimitedValue(
        target_pose_.x, delta, pose_limit_.x, -pose_limit_.x);
  } else if (term == Term::kY) {
    target_pose_.y = Utils::UpdateRangeLimitedValue(
        target_pose_.y, delta, pose_limit_.y, -pose_limit_.y);
  } else if (term == Term::kZ) {
    target_pose_.z = Utils::UpdateRangeLimitedValue(
        target_pose_.z, delta, pose_limit_.z, -pose_limit_.z);
  } else if (term == Term::kYaw) {
    target_pose_.yaw = Utils::UpdateRangeLimitedValue(
        target_pose_.yaw, delta, pose_limit_.yaw, -pose_limit_.yaw);
    target_pose_.quat =
        Eigen::AngleAxisd(Utils::DegreeToRadian(target_pose_.yaw),
                          Eigen::Vector3d::UnitZ()) *
        quat0_;
  }
}

void BalanceTestMode::HandleKeyboardInput(ControlContext& context) {
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kControlInput);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kLeftStickUp) {
      XLOG_DEBUG("FreeStandMode: LeftStickUp");
      UpdateTargetPose(Term::kZ, move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickDown) {
      XLOG_DEBUG("FreeStandMode: LeftStickDown");
      UpdateTargetPose(Term::kZ, -move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickLeft) {
      XLOG_DEBUG("FreeStandMode: LeftStickLeft");
      UpdateTargetPose(Term::kYaw, rotate_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickRight) {
      XLOG_DEBUG("FreeStandMode: LeftStickRight");
      UpdateTargetPose(Term::kYaw, -rotate_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickUp) {
      XLOG_DEBUG("FreeStandMode: RightStickUp");
      UpdateTargetPose(Term::kX, move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickDown) {
      XLOG_DEBUG("FreeStandMode: RightStickDown");
      UpdateTargetPose(Term::kX, -move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickLeft) {
      XLOG_DEBUG("FreeStandMode: RightStickLeft");
      UpdateTargetPose(Term::kY, -move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickRight) {
      XLOG_DEBUG("FreeStandMode: RightStickRight");
      UpdateTargetPose(Term::kY, move_step_);
    }
  }
}

void BalanceTestMode::Update(ControlContext& context) {
  HandleKeyboardInput(context);

  // get current position/velocity and orientation
  Position3d p_b = context.estimator->GetEstimatedBasePosition();
  Velocity3d p_dot_b = context.estimator->GetEstimatedBaseVelocity();
  Quaterniond quat_b = context.estimator->GetEstimatedBaseOrientation();

  // calculate desired acceleration p_ddot
  Eigen::Matrix<double, 3, 1> dp = {target_pose_.x - p_b.x(),
                                    target_pose_.y - p_b.y(),
                                    target_pose_.z - p_b.z()};
  Eigen::Matrix<double, 3, 1> dp_dot =
      Eigen::Matrix<double, 3, 1>::Zero() - p_dot_b;
  Eigen::Matrix<double, 3, 1> p_ddot = pos_ctrl_gains_.kp.asDiagonal() * dp +
                                       pos_ctrl_gains_.kd.asDiagonal() * dp_dot;

  // calculate desired orientation
//  Eigen::Matrix<double, 3, 1> w_dot = ori_ctrl_gains_.kp *

  // finally send the command to the robot
  context.robot_model->SendCommandToRobot();
}

}  // namespace xmotion