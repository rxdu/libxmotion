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
class Ddsm210Array {
 public:
  using Mode = Ddsm210::Mode;

 public:
  Ddsm210Array(std::string dev_name);

  // do not allow copy
  Ddsm210Array(const Ddsm210Array &) = delete;
  Ddsm210Array &operator=(const Ddsm210Array &) = delete;

  // public methods
  void RegisterMotor(uint8_t id);
  void UnregisterMotor(uint8_t id);

  bool Connect();
  void Disconnect();

  Mode GetMode(uint8_t id) const;
  int32_t GetEncoderCount(uint8_t id) const;

  void RequestOdometryFeedback(uint8_t id);
  void RequestModeFeedback(uint8_t id);

  void SetSpeed(uint8_t id, float rpm);
  float GetSpeed(uint8_t id);

  void SetPosition(uint8_t id, float position);
  float GetPosition(uint8_t id);

  void ApplyBrake(uint8_t id, float brake = 1.0);
  void ReleaseBrake(uint8_t id);
  bool IsNormal(uint8_t id);

 private:
  void ProcessFeedback(uint8_t *data, const size_t bufsize, size_t len);

  std::shared_ptr<SerialInterface> serial_;
  std::unordered_map<uint8_t, std::shared_ptr<Ddsm210>> motors_;
};
}  // namespace xmotion

#endif  // XMOTION_DDSM_210_ARRAY_HPP