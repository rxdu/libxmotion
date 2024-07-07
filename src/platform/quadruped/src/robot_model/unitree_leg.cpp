/*
 * unitree_leg.cpp
 *
 * Created on 7/7/24 1:07 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/robot_model/unitree_leg.hpp"

namespace xmotion {
UnitreeLeg::UnitreeLeg(UnitreeModelProfile profile, LegIndex index)
    : index_(index) {
  // set leg link lengths
  if (index == LegIndex::kFrontRight || index == LegIndex::kRearRight) {
    l1_ = -profile.leg_hip_link;
  } else {
    l1_ = profile.leg_hip_link;
  }
  l2_ = -profile.leg_thigh_link;
  l3_ = -profile.leg_calf_link;
}

Position3d UnitreeLeg::GetFootPosition(const JointPosition3d& q) const {
  Position3d pos;
  pos.x() = l3_ * std::cos(q[1] + q[2]) + l2_ * std::sin(q[1]);
  pos.y() = -l3_ * std::sin(q[0]) * std::cos(q[1] + q[2]) +
            l1_ * std::cos(q[0]) - l2_ * std::cos(q[1]) * std::sin(q[0]);
  pos.z() = l3_ * std::cos(q[0]) * std::cos(q[1] + q[2]) +
            l1_ * std::sin(q[0]) + l2_ * std::cos(q[0]) * std::cos(q[1]);
  return pos;
}

Velocity3d UnitreeLeg::GetFootVelocity(const JointPosition3d& q,
                                       const JointVelocity3d& q_dot) const {
  return GetJacobian(q) * q_dot;
}

Eigen::Matrix3d UnitreeLeg::GetJacobian(const JointPosition3d& q) const {
  Eigen::Matrix3d J;
  J(0, 0) = 0;
  J(1, 0) = -l3_ * std::cos(q[0]) * std::cos(q[1] + q[2]) -
            l2_ * std::cos(q[0]) * std::cos(q[1]) - l1_ * std::sin(q[0]);
  J(2, 0) = -l3_ * std::sin(q[0]) * std::cos(q[1] + q[2]) -
            l2_ * std::cos(q[1]) * std::sin(q[0]) + l1_ * std::cos(q[0]);
  J(0, 1) = l3_ * std::cos(q[1] + q[2]) + l2_ * std::cos(q[1]);
  J(1, 1) = l3_ * std::sin(q[0]) * std::sin(q[1] + q[2]) +
            l2_ * std::sin(q[0]) * std::sin(q[1]);
  J(2, 1) = -l3_ * std::cos(q[0]) * std::sin(q[1] + q[2]) -
            l2_ * std::cos(q[0]) * std::sin(q[1]);
  J(0, 2) = l3_ * std::cos(q[1] + q[2]);
  J(1, 2) = l3_ * std::sin(q[0]) * std::sin(q[1] + q[2]);
  J(2, 2) = -l3_ * std::cos(q[0]) * std::sin(q[1] + q[2]);
  return J;
}

JointPosition3d UnitreeLeg::GetJointPosition(const Position3d& pos) const {
  JointPosition3d q;

  // calculate q1
  double L = std::sqrt(pos.y() * pos.y() + pos.z() * pos.z() - l1_ * l1_);
  q.x() = std::atan2(pos.z() * l1_ + pos.y() * L, pos.y() * l1_ - pos.z() * L);

  // calculate q3
  double pos_len_square =
      std::sqrt(pos.x() * pos.x() + pos.y() * pos.y() + pos.z() * pos.z());
  double temp = (l2_ * l2_ + l3_ * l3_ - (pos_len_square - l1_ * l1_)) /
                (2 * std::fabs(l2_ * l3_));
  if (temp > 1.0) temp = 1.0;
  if (temp < -1.0) temp = -1.0;
  q.z() = -M_PI + std::acos(temp);

  // calculate q2
  double m1 = l3_ * std::sin(q[2]);
  double m2 = l2_ + l3_ * std::cos(q[2]);
  double a1 = pos.y() * std::sin(q[0]) - pos.z() * std::cos(q[0]);
  double a2 = pos.x();
  q.y() = std::atan2(a1 * m1 + a2 * m2, a2 * m1 - a1 * m2);

  return q;
}

JointVelocity3d UnitreeLeg::GetJointVelocity(const Position3d& pos,
                                             const Velocity3d& vel) const {
  JointPosition3d q = GetJointPosition(pos);
  return GetJacobian(q).inverse() * vel;
}

JointVelocity3d UnitreeLeg::GetJointVelocityQ(const JointPosition3d& q,
                                              const Velocity3d& vel) const {
  return GetJacobian(q).inverse() * vel;
}

Torque3d UnitreeLeg::GetFootTorque(const JointPosition3d& q,
                                   const Force3d& f) const {
  return GetJacobian(q).transpose() * f;
}
}  // namespace xmotion