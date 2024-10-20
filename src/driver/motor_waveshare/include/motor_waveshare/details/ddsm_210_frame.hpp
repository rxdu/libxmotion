/*
 * @file ddsm_210_frame.hpp
 * @date 10/19/24
 * @brief this class handles encoding and decoding of DDSM-210 serial frames
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_DDSM_210_FRAME_HPP
#define XMOTION_DDSM_210_FRAME_HPP

#include <array>
#include <vector>
#include <cstdint>

namespace xmotion {
class Ddsm210Frame {
 public:
  enum class Mode { kOpenLoop = 0, kSpeed, kPosition };

  enum class Type : uint32_t {
    kUnknown = 0,
    kCommand,
    kSpeedFeedback,        // 0x64
    kOdomFeedback,         // 0x74
    kModeRequestFeedback,  // 0x75
    kModeSwitchFeedback,   // 0xa0
  };

  struct RawFeedback {
    struct {
      int16_t rpm = 0;
      int16_t current = 0;
      uint8_t ms_per_rpm = 0;
      int8_t temperature = 0;
    } speed_feedback;

    struct {
      int32_t encoder_count = 0;
      uint16_t position = 0;
      uint8_t error_code = 0;
    } odom_feedback;

    struct {
      uint8_t mode = 0;
    } mode_request_feedback;

    struct {
      uint8_t mode = 0;
    } mode_switch_feedback;
  };

 public:
  Ddsm210Frame(uint8_t id);
  Ddsm210Frame(uint8_t id, const std::vector<uint8_t>& frame_buffer);

  bool IsValid() const;

  // getters
  Type GetType() const;
  RawFeedback GetRawFeedback() const;

  // setters
  void SetSpeed(float rpm);
  void SetPosition(float position);
  void ApplyBrake();
  void ReleaseBrake();
  void SetId(uint8_t id);
  void SetMode(Mode mode);
  void RequestOdom();
  void RequestMode();
  std::array<uint8_t, 10> ToBuffer();

 private:
  static constexpr uint8_t over_temp_error_bit = 0x10;
  static constexpr uint8_t over_current_error_bit = 0x02;
  static constexpr int16_t max_rpm = 210;
  static constexpr int16_t min_rpm = -210;
  static constexpr uint16_t max_pos = 360;
  static constexpr uint16_t min_pos = 0;

  bool valid_ = false;
  Type type_ = Type::kCommand;
  uint8_t motor_id_;
  std::array<uint8_t, 10> frame_buffer_;
  RawFeedback raw_feedback_;
};
}  // namespace xmotion

#endif  // XMOTION_DDSM_210_FRAME_HPP