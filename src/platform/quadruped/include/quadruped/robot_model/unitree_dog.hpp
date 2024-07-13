/*
 * @file unitree_dog.hpp
 * @date 7/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_UNITREE_DOG_HPP
#define QUADRUPED_MOTION_UNITREE_DOG_HPP

#include <eigen3/Eigen/Core>

#include <array>
#include <mutex>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "quadruped/robot_model/quadruped_model.hpp"
#include "quadruped/robot_model/unitree_leg.hpp"
#include "quadruped/robot_model/unitree_model_profile.hpp"

namespace xmotion {
class UnitreeDog : public QuadrupedModel {
  static constexpr auto low_level_cmd_topic = "rt/lowcmd";
  static constexpr auto low_level_state_topic = "rt/lowstate";

  using State = QuadrupedModel::State;

 public:
  using LowLevelCmd = unitree_go::msg::dds_::LowCmd_;
  using LowLevelState = unitree_go::msg::dds_::LowState_;

 public:
  UnitreeDog(uint32_t domain_id, const std::string& network_interface,
             const UnitreeModelProfile& profile);

  // QuadrupedModel: control interface
  void SetJointGains(const AllJointGains& gains) override;
  void SetJointCommand(const Command& cmd) override;
  void SendCommandToRobot() override;

  // QuadrupedModel: estimator interface
  State GetEstimatedState() override;

  // QuadrupedModel: kinematics interface
  Position3d GetFootPosition(LegIndex leg_index, const JointPosition3d& q,
                             RefFrame frame) const override;
  Velocity3d GetFootVelocity(LegIndex leg_index, const JointPosition3d& q,
                             const JointVelocity3d& q_dot) const override;
  JointPosition3d GetJointPosition(LegIndex leg_index,
                                   const Position3d& pos) const override;
  JointVelocity3d GetJointVelocity(LegIndex leg_index, const Position3d& pos,
                                   const Velocity3d& vel) const override;
  JointVelocity3d GetJointVelocityQ(LegIndex leg_index,
                                    const JointPosition3d& q,
                                    const Velocity3d& vel) const override;

  // QuadrupedModel: dynamics interface
  Torque3d GetJointTorque(LegIndex leg_index, const Position3d& pos,
                          const Force3d& f) const override;
  Torque3d GetJointTorqueQ(LegIndex leg_index, const JointPosition3d& q,
                           const Force3d& f) const override;

 private:
  // unitree-specific implementation details
  void InitCommand();
  void OnLowLevelStateMessageReceived(const void* message);

  std::string network_interface_;
  UnitreeModelProfile profile_;
  std::unordered_map<LegIndex, UnitreeLeg> legs_;
  std::unordered_map<LegIndex, Position3d> body_to_leg_offsets_;

  std::mutex state_mutex_;
  LowLevelState state_;
  LowLevelCmd cmd_;

  unitree::robot::ChannelPublisherPtr<LowLevelCmd> cmd_pub_;
  unitree::robot::ChannelSubscriberPtr<LowLevelState> state_sub_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UNITREE_DOG_HPP
