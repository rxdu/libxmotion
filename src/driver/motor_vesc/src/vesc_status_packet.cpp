/* 
 * vesc_status_packet.cpp
 *
 * Created on 6/30/22 10:51 PM
 * Description:
 *
 * Reference:
 * [1] https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can
 * [2] comm/comm_can.c in https://github.com/vedderb/bldc.git
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "motor_vesc/vesc_status_packet.hpp"

namespace xmotion {
namespace {
float ToFloat(uint8_t byte0, uint8_t byte1) {
  int16_t value = static_cast<int16_t>((static_cast<uint16_t>(byte0) << 8) | 
        static_cast<uint16_t>(byte1));
  return value;
}

float ToFloat(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3) {
  int32_t value = static_cast<int32_t>((static_cast<uint32_t>(byte0) << 24) | 
        (static_cast<uint32_t>(byte1) << 16) | 
        (static_cast<uint32_t>(byte2) << 8) | 
        static_cast<uint32_t>(byte3));
  return value;
}
}

VescStatus1Packet::VescStatus1Packet(const struct can_frame &frame) : VescFrame(frame) {
  // rpm(4 byte), current*10.0(2 byte), duty*1000.0(2 byte)
  uint32_t rpm_raw = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | (frame.data[3]);
  rpm_ = rpm_raw;

//   uint16_t current_raw = (frame.data[4] << 8) | (frame.data[5]);
//   current_ = current_raw / 10.0f;
  current_ = ToFloat(frame.data[4], frame.data[5]) / 10.0f;

  uint16_t duty_raw = (frame.data[6] << 8) | (frame.data[7]);
  duty_ = duty_raw / 1000.0f;
}

VescStatus2Packet::VescStatus2Packet(const struct can_frame &frame) : VescFrame(frame) {
  //  amp_hours*10000.0(4 byte), amp_hours_charged*10000.0(4 byte)
//   uint32_t amp_hours_raw = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | (frame.data[3]);
  amp_hours_ = ToFloat(frame.data[0], frame.data[1], frame.data[2], frame.data[3]) / 10000.0f;

//   uint32_t
//       amp_hours_charged_raw = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | (frame.data[7]);
  amp_hours_charged_ = ToFloat(frame.data[4], frame.data[5], frame.data[6], frame.data[7]) / 10000.0f;
}

VescStatus3Packet::VescStatus3Packet(const struct can_frame &frame) : VescFrame(frame) {
  // watt_hours*10000.0(4 byte), watt_hours_charged*10000.0(4 byte)
//   uint32_t watt_hours_raw = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | (frame.data[3]);
  watt_hours_ = ToFloat(frame.data[0], frame.data[1], frame.data[2], frame.data[3]) / 10000.0f;

  uint32_t
      watt_hours_charged_raw = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | (frame.data[7]);
  watt_hours_charged_ = ToFloat(frame.data[4], frame.data[5], frame.data[6], frame.data[7]) / 10000.0f;
}

VescStatus4Packet::VescStatus4Packet(const struct can_frame &frame) : VescFrame(frame) {
  // temp_fet*10.0(2 byte), temp_motor*10.0(2 byte), current_in*10.0(2 byte), pid_pos_now*50.0(2 byte)
//   uint16_t temp_fet_raw = (frame.data[0] << 8) | (frame.data[1]);
  temp_fet_ = ToFloat(frame.data[0], frame.data[1]) / 10.0f;

//   uint16_t temp_motor_raw = (frame.data[2] << 8) | (frame.data[3]);
  temp_motor_ = ToFloat(frame.data[2], frame.data[3]) / 10.0f;

//   uint16_t current_in_raw = (frame.data[4] << 8) | (frame.data[5]);
  current_in_ = ToFloat(frame.data[4], frame.data[5]) / 10.0f;

//   uint16_t pid_pos_now_raw = (frame.data[6] << 8) | (frame.data[7]);
  pid_pos_now_ = ToFloat(frame.data[6], frame.data[7]) / 50.0f;
}

VescStatus5Packet::VescStatus5Packet(const struct can_frame &frame) : VescFrame(frame) {
  // tacho_value(4 byte), v_in*10.0(2 byte), reserved as 0(2 byte)
  uint32_t tacho_raw = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | (frame.data[3]);
  tacho_value_ = static_cast<int32_t>(tacho_raw);

//   uint16_t vin_raw = (frame.data[4] << 8) | (frame.data[5]);
  voltage_in_ = ToFloat(frame.data[4], frame.data[5]) / 10.0f;
};

VescStatus6Packet::VescStatus6Packet(const struct can_frame &frame) : VescFrame(frame) {

}
}