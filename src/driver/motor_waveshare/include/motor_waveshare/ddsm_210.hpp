/*
 * @file ddsm_210.hpp
 * @date 10/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_DDSM_210_HPP
#define XMOTION_DDSM_210_HPP

#include <string>
#include <cstdint>
#include <memory>

#include "interface/driver/motor_controller_interface.hpp"
#include "interface/driver/serial_interface.hpp"

#include "async_port/ring_buffer.hpp"
#include "motor_waveshare/details/ddsm_210_frame.hpp"

namespace xmotion {
class Ddsm210 : public MotorControllerInterface {
 public:
  using Mode = Ddsm210Frame::Mode;

 public:
  explicit Ddsm210(uint8_t id);
  // this construction is used for motor array
  Ddsm210(uint8_t id, std::shared_ptr<SerialInterface> serial);
  ~Ddsm210();

  // do not allow copy
  Ddsm210(const Ddsm210&) = delete;
  Ddsm210& operator=(const Ddsm210&) = delete;

  // public methods
  bool Connect(std::string dev_name);
  void Disconnect();

  Mode GetMode() const;
  int32_t GetEncoderCount() const;

  void RequestOdometryFeedback();
  void RequestModeFeedback();

  void SetSpeed(float rpm) override;
  float GetSpeed() override;

  void SetPosition(float position) override;
  float GetPosition() override;

  void ApplyBrake(float brake = 1.0) override;
  void ReleaseBrake() override;
  bool IsNormal() override;

  // the following functions may not be called during normal motor operation
  // in most cases, motor id and mode should be set beforehand
  bool SetMode(Mode mode, uint32_t timeout_ms = 100);
  bool SetMotorId(uint8_t id, uint32_t timeout_ms = 100);

 private:
  friend class Ddsm210Array;
  void ProcessFeedback(uint8_t* data, const size_t bufsize, size_t len);

  uint8_t motor_id_;
  std::shared_ptr<SerialInterface> serial_;
  RingBuffer<uint8_t, 1024> rx_buffer_;
  Ddsm210Frame::RawFeedback raw_feedback_;
  bool id_set_ack_received_ = false;
};
}  // namespace xmotion

#endif  // XMOTION_DDSM_210_HPP