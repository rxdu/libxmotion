/*
 * free_stand_mode.cpp
 *
 * Created on 7/4/24 11:30 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/free_stand_mode.hpp"

#include "quadruped/utils.hpp"
#include "quadruped/matrix_helper.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
FreeStandMode::FreeStandMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to FreeStandMode");

  context.robot_model->SetJointGains(
      context.system_config.ctrl_settings.fixed_stand_mode.default_joint_gains);

  joint_cmd_.q_dot = QuadrupedModel::AllJointVar::Zero();
  joint_cmd_.tau = QuadrupedModel::AllJointVar::Zero();

  pose_limit_ = context.system_config.ctrl_settings.free_stand_mode.pose_limit;
  angle_step_ = context.system_config.ctrl_settings.free_stand_mode.angle_step;
  height_step_ =
      context.system_config.ctrl_settings.free_stand_mode.height_step;

  XLOG_INFO("FreeStandMode: change limit (r,p,y,z): {}, {}, {}, {}",
            pose_limit_.roll, pose_limit_.pitch, pose_limit_.yaw,
            pose_limit_.height);
  XLOG_INFO("FreeStandMode: step size (angle, height): {}, {}", angle_step_,
            height_step_);

  // front right foot position with respect to base
  auto current_state = context.estimator->GetCurrentState();
  JointPosition3d q_b0 =
      current_state.q.segment<3>(static_cast<int>(LegIndex::kFrontRight) * 3);
  p_b0_ = context.robot_model->GetFootPosition(LegIndex::kFrontRight, q_b0,
                                               QuadrupedModel::RefFrame::kBase);

  initial_pose_.height = -p_b0_.z();
  target_pose_.height = initial_pose_.height;

  // foot position with respect to front right foot
  auto q_b1 =
      current_state.q.segment<3>(static_cast<int>(LegIndex::kFrontLeft) * 3);
  auto q_b2 =
      current_state.q.segment<3>(static_cast<int>(LegIndex::kRearRight) * 3);
  auto q_b3 =
      current_state.q.segment<3>(static_cast<int>(LegIndex::kRearLeft) * 3);
  p_sx_[0] = p_b0_ - p_b0_;
  p_sx_[1] = context.robot_model->GetFootPosition(
                 LegIndex::kFrontLeft, q_b1, QuadrupedModel::RefFrame::kBase) -
             p_b0_;
  p_sx_[2] = context.robot_model->GetFootPosition(
                 LegIndex::kRearRight, q_b2, QuadrupedModel::RefFrame::kBase) -
             p_b0_;
  p_sx_[3] = context.robot_model->GetFootPosition(
                 LegIndex::kRearLeft, q_b3, QuadrupedModel::RefFrame::kBase) -
             p_b0_;

  XLOG_INFO("FreeStandMode: initial foot pose (r,p,y,z): {}, {}, {}, {}",
            initial_pose_.roll, initial_pose_.pitch, initial_pose_.yaw,
            initial_pose_.height);
}

void FreeStandMode::UpdateTargetPose(Term term, double delta) {
  if (term == Term::kRoll) {
    target_pose_.roll = Utils::UpdateRangeLimitedValue(
        target_pose_.roll, delta, -pose_limit_.roll, pose_limit_.roll);
  } else if (term == Term::kPitch) {
    target_pose_.pitch = Utils::UpdateRangeLimitedValue(
        target_pose_.pitch, delta, -pose_limit_.pitch, pose_limit_.pitch);
  } else if (term == Term::kYaw) {
    target_pose_.yaw = Utils::UpdateRangeLimitedValue(
        target_pose_.yaw, delta, -pose_limit_.yaw, pose_limit_.yaw);
  } else if (term == Term::kHeight) {
    target_pose_.height = Utils::UpdateChangeLimitedValue(
        target_pose_.height, initial_pose_.height, delta, -pose_limit_.height,
        pose_limit_.height);
  }

  XLOG_INFO("FreeStandMode: target pose (r,p,y,z): {}, {}, {}, {}",
            target_pose_.roll, target_pose_.pitch, target_pose_.yaw,
            target_pose_.height);
}

void FreeStandMode::HandleKeyboardInput(ControlContext& context) {
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kControlInput);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kLeftStickUp) {
      XLOG_DEBUG("FreeStandMode: LeftStickUp");
      UpdateTargetPose(Term::kHeight, height_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickDown) {
      XLOG_DEBUG("FreeStandMode: LeftStickDown");
      UpdateTargetPose(Term::kHeight, -height_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickLeft) {
      XLOG_DEBUG("FreeStandMode: LeftStickLeft");
      UpdateTargetPose(Term::kYaw, angle_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickRight) {
      XLOG_DEBUG("FreeStandMode: LeftStickRight");
      UpdateTargetPose(Term::kYaw, -angle_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickUp) {
      XLOG_DEBUG("FreeStandMode: RightStickUp");
      UpdateTargetPose(Term::kPitch, angle_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickDown) {
      XLOG_DEBUG("FreeStandMode: RightStickDown");
      UpdateTargetPose(Term::kPitch, -angle_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickLeft) {
      XLOG_DEBUG("FreeStandMode: RightStickLeft");
      UpdateTargetPose(Term::kRoll, -angle_step_);
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickRight) {
      XLOG_DEBUG("FreeStandMode: RightStickRight");
      UpdateTargetPose(Term::kRoll, angle_step_);
    }
  }
}

void FreeStandMode::Update(ControlContext& context) {
  HandleKeyboardInput(context);

  // calculate target foot position
  Position3d p_0 = -p_b0_;
  p_0(2) = target_pose_.height;

  // convert from degree to radian
  double roll = Utils::DegreeToRadian(target_pose_.roll);
  double pitch = Utils::DegreeToRadian(target_pose_.pitch);
  double yaw = Utils::DegreeToRadian(target_pose_.yaw);

  RotMatrix3d R = MatrixHelper::RpyToRotMatrix(roll, pitch, yaw);
  HomoMatrix3d T_sb = MatrixHelper::CreateHomoMatrix(R, p_0);
  HomoMatrix3d T_bs = MatrixHelper::GetHomoMatrixInverse(T_sb);

  std::array<Position3d, 4> p_bx;
  for (int i = 0; i < 4; ++i) {
    p_bx[i] = MatrixHelper::ApplyHomoMatrix(T_bs, p_sx_[i]);
  }
  joint_cmd_.q = context.robot_model->GetJointPosition(
      p_bx, QuadrupedModel::RefFrame::kBase);

  context.robot_model->SetJointCommand(joint_cmd_);

  // finally send the command to the robot
  context.robot_model->SendCommandToRobot();
}
}  // namespace xmotion