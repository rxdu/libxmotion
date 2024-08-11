/*
 * swing_test_mode.cpp
 *
 * Created on 7/12/24 10:16 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/swing_test_mode.hpp"

#include "quadruped/utils.hpp"
#include "logging/xlogger.hpp"

namespace xmotion {
SwingTestMode::SwingTestMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to SwingTestMode");

  swing_leg_ =
      context.system_config.ctrl_settings.swing_test_mode.swing_leg_index;
  change_limit_ =
      context.system_config.ctrl_settings.swing_test_mode.change_limit;
  move_step_ = context.system_config.ctrl_settings.swing_test_mode.move_step;

  // default joint gains
  auto default_joint_gains =
      context.system_config.ctrl_settings.swing_test_mode.default_joint_gains;
  default_joint_gains.kp.segment<3>(static_cast<int>(swing_leg_) * 3) =
      context.system_config.ctrl_settings.swing_test_mode.swing_leg_gains.kp;
  default_joint_gains.kd.segment<3>(static_cast<int>(swing_leg_) * 3) =
      context.system_config.ctrl_settings.swing_test_mode.swing_leg_gains.kd;
  context.robot_model->SetJointGains(default_joint_gains);

  // foot swing control gains
  kp_ = context.system_config.ctrl_settings.swing_test_mode.kp.asDiagonal();
  kd_ = context.system_config.ctrl_settings.swing_test_mode.kd.asDiagonal();

  // initialize command
  joint_cmd_.q = QuadrupedModel::AllJointVar::Zero();
  joint_cmd_.q_dot = QuadrupedModel::AllJointVar::Zero();
  joint_cmd_.tau = QuadrupedModel::AllJointVar::Zero();
  context.robot_model->SetJointCommand(joint_cmd_);

  auto q_hat = context.estimator->GetEstimatedJointPosition();

  auto foot_joints = q_hat.segment<3>(static_cast<int>(swing_leg_) * 3);
  initial_position_ = context.robot_model->GetFootPosition(
      swing_leg_, foot_joints, QuadrupedModel::RefFrame::kLeg);
  target_position_ = initial_position_;
  XLOG_INFO("SwingTestMode: initial swing leg {} foot position: {}, {}, {}",
            static_cast<int>(swing_leg_), initial_position_.x(),
            initial_position_.y(), initial_position_.z());
}

void SwingTestMode::UpdateTargetFootPosition(Axis axis, double delta) {
  if (axis == Axis::kX) {
    double change = target_position_.x() + delta - initial_position_.x();
    if (change >= change_limit_.x_min && change <= change_limit_.x_max) {
      target_position_.x() += delta;
    }
  } else if (axis == Axis::kY) {
    double change = target_position_.y() + delta - initial_position_.y();
    if (change >= change_limit_.y_min && change <= change_limit_.y_max) {
      target_position_.y() += delta;
    }
  } else if (axis == Axis::kZ) {
    double change = target_position_.z() + delta - initial_position_.z();
    if (change >= change_limit_.z_min && change <= change_limit_.z_max) {
      target_position_.z() += delta;
    }
  }
}

void SwingTestMode::HandleKeyboardInput(ControlContext& context) {
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kControlInput);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kLeftStickUp) {
      XLOG_INFO("SwingTestMode: LeftStickUp");
      UpdateTargetFootPosition(Axis::kZ, move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickDown) {
      XLOG_INFO("SwingTestMode: LeftStickDown");
      UpdateTargetFootPosition(Axis::kZ, -move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickLeft) {
      XLOG_INFO("SwingTestMode: LeftStickLeft");
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickRight) {
      XLOG_INFO("SwingTestMode: LeftStickRight");
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickUp) {
      XLOG_INFO("SwingTestMode: RightStickUp");
      UpdateTargetFootPosition(Axis::kX, move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickDown) {
      XLOG_INFO("SwingTestMode: RightStickDown");
      UpdateTargetFootPosition(Axis::kX, -move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickLeft) {
      XLOG_INFO("SwingTestMode: RightStickLeft");
      UpdateTargetFootPosition(Axis::kY, move_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickRight) {
      XLOG_INFO("SwingTestMode: RightStickRight");
      UpdateTargetFootPosition(Axis::kY, -move_step_);
    }
  }

  //  XLOG_INFO("SwingTestMode: target foot position: {}, {}, {}",
  //            target_position_.x(), target_position_.y(),
  //            target_position_.z());
}

void SwingTestMode::Update(ControlContext& context) {
  //  sw_.tic();
  HandleKeyboardInput(context);
  //  if (sw_.mtoc() > 0) XLOG_INFO("Updating keyboard took: {}", sw_.mtoc());

  auto q_hat = context.estimator->GetEstimatedJointPosition();
  auto q_dot_hat = context.estimator->GetEstimatedJointVelocity();

  // desired joint position
  auto swing_q_desired =
      context.robot_model->GetJointPosition(swing_leg_, target_position_);
  joint_cmd_.q = q_hat;
  joint_cmd_.q.segment<3>(static_cast<int>(swing_leg_) * 3) = swing_q_desired;

  // calculate desired joint torque
  auto q_foot = q_hat.segment<3>(static_cast<int>(swing_leg_) * 3);
  auto q_dot_foot = q_dot_hat.segment<3>(static_cast<int>(swing_leg_) * 3);

  auto pos_foot = context.robot_model->GetFootPosition(
      swing_leg_, q_foot, QuadrupedModel::RefFrame::kLeg);
  auto vel_foot =
      context.robot_model->GetFootVelocity(swing_leg_, q_foot, q_dot_foot);

  auto target_force = kp_ * (target_position_ - pos_foot) + kd_ * (-vel_foot);
  auto target_torque =
      context.robot_model->GetJointTorqueQ(swing_leg_, q_foot, target_force);
  //  joint_cmd_.tau = current_state.tau;
  joint_cmd_.tau = QuadrupedModel::AllJointVar::Zero();
  joint_cmd_.tau.segment<3>(static_cast<int>(swing_leg_) * 3) += target_torque;

  //  XLOG_INFO("==> SwingTestMode: target foot torque: {}, {}, {}",
  //            target_torque.x(), target_torque.y(), target_torque.z());

  context.robot_model->SetJointCommand(joint_cmd_);

  // finally send the command to the robot
  context.robot_model->SendCommandToRobot();
}
}  // namespace xmotion