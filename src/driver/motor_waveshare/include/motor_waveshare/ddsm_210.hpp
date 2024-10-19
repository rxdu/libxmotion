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
  enum class Mode { kOpenLoop, kSpeed, kPosition };

 public:
  Ddsm210(uint8_t id);
  ~Ddsm210() = default;

  // do not allow copy
  Ddsm210(const Ddsm210&) = delete;
  Ddsm210& operator=(const Ddsm210&) = delete;

  // public methods
  bool Connect(std::string dev_name);
  void Disconnect();

  Mode GetMode() const;
  int32_t GetEncoderCount();

  bool SetMode(Mode mode, uint32_t timeout_ms = 100);
  bool SetMotorId(uint8_t id, uint32_t timeout_ms = 100);
  void SetAcceleration(uint8_t ms_per_rpm);

  void SetSpeed(int32_t rpm) override;
  int32_t GetSpeed() override;

  void SetPosition(double position) override;
  double GetPosition() override;

  void ApplyBrake(double brake = true) override;
  void ReleaseBrake() override;
  bool IsNormal() override;

 private:
  void RequestOdometryFeedback();
  void ProcessFeedback(uint8_t* data, const size_t bufsize, size_t len);

  uint8_t motor_id_;
  std::shared_ptr<SerialInterface> serial_;
  RingBuffer<uint8_t, 1024> rx_buffer_;
  Ddsm210Frame::RawFeedback raw_feedback_;

  uint8_t ms_per_rpm_ = 1;
  bool id_set_ack_received_ = false;
  bool mode_set_ack_received_ = false;
};
}  // namespace xmotion

#endif  // XMOTION_DDSM_210_HPP