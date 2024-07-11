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

  void SetJointGains(const JointGains& gains) override;
  void SetTargetState(const State& state) override;
  State GetEstimatedState() override;
  void SendCommandToRobot() override;

 private:
  void InitCommand();
  void OnLowLevelStateMessageReceived(const void* message);

  std::string network_interface_;
  UnitreeModelProfile profile_;
  std::unordered_map<LegIndex, UnitreeLeg> legs_;

  std::mutex state_mutex_;
  LowLevelState state_;
  LowLevelCmd cmd_;

  unitree::robot::ChannelPublisherPtr<LowLevelCmd> cmd_pub_;
  unitree::robot::ChannelSubscriberPtr<LowLevelState> state_sub_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UNITREE_DOG_HPP
