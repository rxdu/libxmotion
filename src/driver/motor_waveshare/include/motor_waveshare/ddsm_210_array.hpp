/*
 * @file ddsm_210_array.hpp
 * @date 10/19/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_DDSM_210_ARRAY_HPP
#define XMOTION_DDSM_210_ARRAY_HPP

#include <memory>

#include "interface/driver/serial_interface.hpp"

namespace xmotion {
class Ddsm210Manager {
 public:
  enum class Mode { kOpenLoop, kSpeed, kPosition };

 public:
  Ddsm210Manager() = default;

  // do not allow copy
  Ddsm210Manager(const Ddsm210Manager &) = delete;
  Ddsm210Manager &operator=(const Ddsm210Manager &) = delete;

  // public methods
  bool Connect(std::string dev_name);
  void Disconnect();

  bool SetMode(Mode mode, uint32_t timeout_ms = 100);
  bool SetMotorId(uint8_t id, uint32_t timeout_ms = 100);
  void SetAcceleration(uint8_t ms_per_rpm);

  void SetSpeed(int32_t rpm);
  int32_t GetSpeed();

  void SetPosition(double position);
  double GetPosition();

  void ApplyBrake(double brake);
  void ReleaseBrake();
  bool IsNormal();

 private:
  std::shared_ptr<SerialInterface> serial_;
};
}  // namespace xmotion

#endif  // XMOTION_DDSM_210_ARRAY_HPP