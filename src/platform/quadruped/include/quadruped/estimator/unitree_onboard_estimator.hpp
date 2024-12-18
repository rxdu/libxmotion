/*
 * @file sim_dummy_estimator.hpp
 * @date 7/23/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_UNITREE_ONBOARD_ESTIMATOR_HPP
#define QUADRUPED_MOTION_UNITREE_ONBOARD_ESTIMATOR_HPP

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "quadruped/robot_model/quadruped_model.hpp"
#include "quadruped/estimator/estimator_interface.hpp"

namespace xmotion {
class UnitreeOnboardEstimator final : public EstimatorInterface {
  static constexpr auto low_level_state_topic = "rt/lowstate";
  static constexpr auto sport_mode_state_topic = "rt/sportmodestate";

  using LowLevelState = unitree_go::msg::dds_::LowState_;
  using SportModeState = unitree_go::msg::dds_::SportModeState_;

  struct EstimatorState {
    QuadrupedModel::AllJointVar q;
    QuadrupedModel::AllJointVar q_dot;
    Position3d p_base;
    Velocity3d v_base;
    Quaterniond quat_base;
  };

 public:
  UnitreeOnboardEstimator();
  ~UnitreeOnboardEstimator() = default;

  void Update(const QuadrupedModel::SensorData& sensor_data,
              double dt) override;

  Eigen::Vector3d GetGyroRaw() const override;
  Eigen::Vector3d GetAccelRaw() const override;

  QuadrupedModel::AllJointVar GetEstimatedJointPosition() const override;
  QuadrupedModel::AllJointVar GetEstimatedJointVelocity() const override;

  Position3d GetEstimatedBasePosition() const override;
  Velocity3d GetEstimatedBaseVelocity() const override;
  Quaterniond GetEstimatedBaseOrientation() const override;

 private:
  void OnLowLevelStateMessageReceived(const void* message);
  void OnSportModeStateMessageReceived(const void* message);

  mutable std::mutex sensor_raw_mutex_;
  QuadrupedModel::SensorData sensor_raw_;

  mutable std::mutex x_hat_mutex_;
  EstimatorState x_hat_;

  mutable std::mutex high_state_mutex_;
  SportModeState high_state_;
  unitree::robot::ChannelSubscriberPtr<SportModeState> high_state_sub_;

  mutable std::mutex low_state_mutex_;
  LowLevelState low_state_;
  unitree::robot::ChannelSubscriberPtr<LowLevelState> low_state_sub_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UNITREE_ONBOARD_ESTIMATOR_HPP