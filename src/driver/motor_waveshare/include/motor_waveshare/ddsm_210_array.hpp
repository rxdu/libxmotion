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
#include "interface/driver/motor_controller_array_interface.hpp"
#include "motor_waveshare/ddsm_210.hpp"

namespace xmotion {
class Ddsm210Array final : public MotorControllerArrayInterface {
 public:
  using Mode = Ddsm210::Mode;

 public:
  Ddsm210Array(std::string dev_name);

  // do not allow copy
  Ddsm210Array(const Ddsm210Array &) = delete;
  Ddsm210Array &operator=(const Ddsm210Array &) = delete;

  // public methods
  void RegisterMotor(uint8_t id) override;
  void UnregisterMotor(uint8_t id) override;

  bool Connect();
  void Disconnect();

  Mode GetMode(uint8_t id) const;
  int32_t GetEncoderCount(uint8_t id) const;

  void RequestOdometryFeedback(uint8_t id);
  void RequestModeFeedback(uint8_t id);

  void SetSpeed(uint8_t id, float rpm) override;
  void SetSpeeds(std::unordered_map<uint8_t, float> speeds) override;
  float GetSpeed(uint8_t id) override;

  void SetPosition(uint8_t id, float position) override;
  float GetPosition(uint8_t id) override;

  void ApplyBrake(uint8_t id, float brake = 1.0) override;
  void ReleaseBrake(uint8_t id) override;
  bool IsNormal(uint8_t id) override;

 private:
  void ProcessFeedback(uint8_t *data, const size_t bufsize, size_t len);

  std::shared_ptr<SerialInterface> serial_;
  std::unordered_map<uint8_t, std::shared_ptr<Ddsm210>> motors_;
};
}  // namespace xmotion

#endif  // XMOTION_DDSM_210_ARRAY_HPP