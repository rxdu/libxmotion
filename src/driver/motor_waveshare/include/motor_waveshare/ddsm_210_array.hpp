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
#include "motor_waveshare/ddsm_210.hpp"

namespace xmotion {
class Ddsm210Manager {
 public:
  using Mode = Ddsm210::Mode;

 public:
  Ddsm210Manager() = default;

  // do not allow copy
  Ddsm210Manager(const Ddsm210Manager &) = delete;
  Ddsm210Manager &operator=(const Ddsm210Manager &) = delete;

  // public methods
  bool Connect(std::string dev_name);
  void Disconnect();

  void SetSpeed(uint8_t id, int32_t rpm);
  int32_t GetSpeed(uint8_t id);

  void SetPosition(uint8_t id, double position);
  double GetPosition(uint8_t id);

  void ApplyBrake(uint8_t id, double brake);
  void ReleaseBrake(uint8_t id);
  bool IsNormal(uint8_t id);

 private:
  std::shared_ptr<SerialInterface> serial_;
  std::unordered_map<uint8_t, std::shared_ptr<Ddsm210>> motors_;
};
}  // namespace xmotion

#endif  // XMOTION_DDSM_210_ARRAY_HPP