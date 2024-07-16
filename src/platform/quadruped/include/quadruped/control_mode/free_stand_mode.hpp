/*
 * @file free_stand_mode.hpp
 * @date 7/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_FREE_STAND_MODE_HPP
#define QUADRUPED_MOTION_FREE_STAND_MODE_HPP

#include <array>

#include "fsm/fsm_template.hpp"
#include "quadruped/control_context.hpp"

namespace xmotion {
class FreeStandMode : public FsmState<ControlContext> {
  enum class Term { kRoll, kPitch, kYaw, kHeight };

  struct Pose {
    // angles in degrees
    double roll;
    double pitch;
    double yaw;
    double height;
  };

 public:
  FreeStandMode(const ControlContext &context);
  ~FreeStandMode() = default;

  void Update(ControlContext &context);

 private:
  void UpdateTargetPose(Term term, double delta);
  void HandleKeyboardInput(ControlContext &context);

  Position3d p_b0_;
  std::array<Position3d, 4> p_sx_;
  Pose initial_pose_{0, 0, 0, 0};
  Pose target_pose_{0, 0, 0, 0};
  double angle_step_;
  double height_step_;
  ControlSettings::FreeStandModeParams::PoseLimit pose_limit_;
  QuadrupedModel::Command joint_cmd_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_FREE_STAND_MODE_HPP
