/*
 * @file speed_actuator_group.hpp
 * @date 4/21/24
 * @brief if motors always receive the same command and used in the same way,
 * then we can group them together and treat them as a single actuator
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SPEED_ACTUATOR_GROUP_HPP_
#define XMOTION_SPEED_ACTUATOR_GROUP_HPP_

#include <vector>
#include <memory>

#include "interface/driver/motor_controller_interface.hpp"
#include "logging/xlogger.hpp"

namespace xmotion {
class SpeedActuatorGroup final : public MotorControllerInterface {
 public:
  SpeedActuatorGroup(
      std::vector<std::shared_ptr<MotorControllerInterface>> actuators)
      : actuators_(actuators) {
    XLOG_DEBUG("speed actuator group created with {} actuators",
               actuators_.size());
  }

  // public interface
  void SetSpeed(float rpm) override;
  float GetSpeed() override;

  void ApplyBrake(float brake) override;
  void ReleaseBrake() override;

  bool IsNormal() override;

 private:
  std::vector<std::shared_ptr<MotorControllerInterface>> actuators_;
};
}  // namespace xmotion

#endif  // XMOTION_SPEED_ACTUATOR_GROUP_HPP_
