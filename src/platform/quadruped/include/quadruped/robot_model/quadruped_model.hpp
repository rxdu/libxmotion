/*
 * quadruped_model.hpp
 *
 * Created on 7/6/24 11:03 PM
 * Description:
 *  A generic class for quadruped robot model with 12DOF (3DOF per leg) with
 *  a similar configuration as the MIT/Unitree robot dog.
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_QUADRUPED_MODEL_HPP
#define QUADRUPED_MOTION_QUADRUPED_MODEL_HPP

#include <array>
#include <memory>

#include "interface/type/geometry_types.hpp"
#include "quadruped/robot_model/data_queue.hpp"

namespace xmotion {
enum class LegIndex : int {
  kFrontRight = 0,
  kFrontLeft = 1,
  kRearRight = 2,
  kRearLeft = 3
};

class QuadrupedModel {
 public:
  using LegJointVar = Eigen::Matrix<double, 3, 1>;
  using AllJointVar = Eigen::Matrix<double, 12, 1>;

  enum class RefFrame { kBase = 0, kLeg };

  struct LegJointGains {
    LegJointVar kp;
    LegJointVar kd;
  };

  struct AllJointGains {
    AllJointVar kp;
    AllJointVar kd;
  };

  struct State {
    AllJointVar q;
    AllJointVar q_dot;
    AllJointVar tau;
    AllJointVar q_ddot;
  };

  struct Command {
    AllJointVar q;
    AllJointVar q_dot;
    AllJointVar tau;
  };

  struct SensorData {
    AllJointVar q;
    Quaterniond quaternion;
    Eigen::Vector3d gyroscope;
    Eigen::Vector3d accelerometer;
  };

 public:
  // estimator
  // TODO (rdu): estimated state should be from a state estimator
  virtual void ConnectSensorDataQueue(
      std::shared_ptr<DataQueue<SensorData>> queue) = 0;
  virtual State GetEstimatedState() = 0;

  // kinematics
  virtual Position3d GetFootPosition(LegIndex leg_index,
                                     const JointPosition3d& q,
                                     RefFrame frame) const = 0;
  virtual Velocity3d GetFootVelocity(LegIndex leg_index,
                                     const JointPosition3d& q,
                                     const JointVelocity3d& q_dot) const = 0;
  virtual JointPosition3d GetJointPosition(LegIndex leg_index,
                                           const Position3d& pos) const = 0;
  virtual JointVelocity3d GetJointVelocity(LegIndex leg_index,
                                           const Position3d& pos,
                                           const Velocity3d& vel) const = 0;
  virtual JointVelocity3d GetJointVelocityQ(LegIndex leg_index,
                                            const JointPosition3d& q,
                                            const Velocity3d& vel) const = 0;

  // full-body kinematics
  virtual AllJointVar GetJointPosition(
      const std::array<Position3d, 4>& foot_pos, RefFrame frame) const = 0;
  virtual AllJointVar GetJointVelocity(
      const std::array<Position3d, 4>& foot_pos,
      const std::array<Velocity3d, 4>& foot_vel, RefFrame frame) const = 0;
  virtual AllJointVar GetJointTorqueQ(
      const AllJointVar& q, const std::array<Force3d, 4>& foot_force) const = 0;

  virtual std::array<Position3d, 4> GetFootPosition(const AllJointVar& q,
                                                    RefFrame frame) const = 0;
  virtual std::array<Velocity3d, 4> GetFootVelocity(
      const AllJointVar& q, const AllJointVar& q_dot) const = 0;

  // dynamics
  virtual Torque3d GetJointTorque(LegIndex leg_index, const Position3d& pos,
                                  const Force3d& f) const = 0;
  virtual Torque3d GetJointTorqueQ(LegIndex leg_index, const JointPosition3d& q,
                                   const Force3d& f) const = 0;

  // control interface
  virtual void SetJointGains(const AllJointGains& gains) = 0;
  virtual void SetJointCommand(const Command& cmd) = 0;
  virtual void SendCommandToRobot() = 0;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_QUADRUPED_MODEL_HPP
