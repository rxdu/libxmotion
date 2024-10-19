/*
 * @file ddsm_210_frame.hpp
 * @date 10/19/24
 * @brief
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
    kIdSetCmd,             // 0x55
    kSpeedCmd,             // 0x64
    kPositionCmd,          // 0x64
    kBrakeCmd,             // 0x64
    kIdSetFeedback,        // 0x64
    kModeSwitchCmd,        // 0xa0
    kModeFeedback,         // 0xa0
    kOdomRequestCmd,       // 0x74
    kOdomFeedback,         // 0x74
    kModeRequestCmd,       // 0x75
    kModeRequestFeedback,  // 0x75
  };

  struct RawFeedback {
    int16_t rpm;
    int16_t current;
    uint8_t ms_per_rpm;
    int8_t temperature;
    int32_t encoder_count;
    int16_t position;
    uint8_t error_code;
    uint8_t mode;
  };

 public:
  Ddsm210Frame(uint8_t id);
  Ddsm210Frame(const std::vector<uint8_t>& frame_buffer);

  bool IsValid() const;

  // getters
  Type GetType() const;
  RawFeedback GetRawFeedback() const;

  // setters
  void SetSpeed(int16_t rpm);
  void SetPosition(int16_t position);
  void ApplyBrake();
  void ReleaseBrake();
  void SetId(uint8_t id);
  void SetMode(Mode mode);
  void RequestOdom();
  std::array<uint8_t, 10> ToBuffer();

 private:
  static constexpr uint8_t over_temp_error_bit = 0x10;
  static constexpr uint8_t over_current_error_bit = 0x02;
  static constexpr int16_t max_rpm = 210;
  static constexpr int16_t min_rpm = -210;
  static constexpr uint16_t max_pos = 360;
  static constexpr uint16_t min_pos = 0;

  bool valid_ = false;
  Type type_;
  uint8_t motor_id_;
  std::array<uint8_t, 10> frame_buffer_;
  RawFeedback raw_feedback_;
};
}  // namespace xmotion

#endif  // XMOTION_DDSM_210_FRAME_HPP