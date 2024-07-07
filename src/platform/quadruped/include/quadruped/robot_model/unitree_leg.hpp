/*
 * @file unitree_leg.hpp
 * @date 7/7/24
 * @brief
 *  A unitree leg consists of 3 actuated joints and 3 links:
 *  - (connected to body)
 *  - [joint1] hip joint (hip abduction/adduction) -> to body left/right
 *  - [link1] hip
 *  - [joint2] shoulder joint (shoulder flexion/extension) -> to body front/back
 *  - [link2] thigh
 *  - [joint3] knee joint (knee flexion/extension) -> to body up/down
 *  - [link3] calf
 *
 *  The legs are indexed as follows:
 *  - front left: 0
 *  - front right: 1
 *  - rear right: 2
 *  - rear left: 3
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_UNITREE_LEG_HPP
#define QUADRUPED_MOTION_UNITREE_LEG_HPP

#include "quadruped/robot_model/quadruped_model.hpp"
#include "quadruped/robot_model/unitree_model_profile.hpp"
#include "quadruped/robot_model/unitree_motor.hpp"
#include "interface/type/geometry_types.hpp"

namespace xmotion {
class UnitreeLeg {
 public:
  UnitreeLeg() = default;
  UnitreeLeg(const UnitreeModelProfile& profile, LegIndex index);

  // commands
  void Enable();
  void Disable();
  void SetFootTarget(const Position3d& pos, const Velocity3d& vel,
                     const Force3d& f);
  void SetJointTarget(const JointPosition3d& q, const JointVelocity3d& q_dot,
                      const Torque3d& tau);
  std::unordered_map<int, UnitreeMotor::CmdMsg> GetMotorCommandMsgs();

  // forward kinematics
  Position3d GetFootPosition(const JointPosition3d& q) const;
  Velocity3d GetFootVelocity(const JointPosition3d& q,
                             const JointVelocity3d& q_dot) const;

  // inverse kinematics
  JointPosition3d GetJointPosition(const Position3d& pos) const;
  JointVelocity3d GetJointVelocity(const Position3d& pos,
                                   const Velocity3d& vel) const;
  JointVelocity3d GetJointVelocityQ(const JointPosition3d& q,
                                    const Velocity3d& vel) const;

  // force/torque calculation
  Torque3d GetJointTorque(const Position3d& pos, const Force3d& f) const;
  Torque3d GetJointTorqueQ(const JointPosition3d& q, const Force3d& f) const;

 private:
  Eigen::Matrix3d GetJacobian(const JointPosition3d& q) const;

  LegIndex index_;
  std::array<UnitreeMotor, 3> motors_;

  double l1_;
  double l2_;
  double l3_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UNITREE_LEG_HPP
