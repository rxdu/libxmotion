/*
 * @file position_actuator_group.hpp
 * @date 11/9/24
 * @brief if motors always receive the same command and used in the same way,
 * then we can group them together and treat them as a single actuator
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_POSITION_ACTUATOR_GROUP_HPP
#define XMOTION_POSITION_ACTUATOR_GROUP_HPP

#include <vector>
#include <memory>

#include "interface/driver/motor_controller_interface.hpp"
#include "logging/xlogger.hpp"

namespace xmotion {
class PositionActuatorGroup final : public MotorControllerInterface {
 public:
  PositionActuatorGroup(
      std::vector<std::shared_ptr<MotorControllerInterface>> actuators)
      : actuators_(actuators) {
    XLOG_DEBUG("position actuator group created with {} actuators",
               actuators_.size());
  }

  // public interface
  void SetPosition(float position) override;
  float GetPosition() override;

  bool IsNormal() override;

 private:
  std::vector<std::shared_ptr<MotorControllerInterface>> actuators_;
};
}  // namespace xmotion

#endif  // XMOTION_POSITION_ACTUATOR_GROUP_HPP