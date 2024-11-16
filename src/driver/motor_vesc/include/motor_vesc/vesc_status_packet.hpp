/* 
 * vesc_status_packet.hpp
 *
 * Created on 6/30/22 10:51 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_STATUS_PACKET_HPP
#define ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_STATUS_PACKET_HPP

#include "motor_vesc/vesc_frame.hpp"

namespace xmotion {
class VescStatus1Packet : public VescFrame {
 public:
  VescStatus1Packet(const struct can_frame &frame);

  float GetRpm() const { return rpm_; };
  float GetCurrent() const { return current_; };
  float GetDuty() const { return duty_; };

 private:
  float rpm_;
  float current_;
  float duty_;
};

class VescStatus2Packet : public VescFrame {
 public:
  VescStatus2Packet(const struct can_frame &frame);

  float GetAmpHours() const { return amp_hours_; }
  float GetAmpHoursCharged() const { return amp_hours_charged_; }

 private:
  float amp_hours_;
  float amp_hours_charged_;
};

class VescStatus3Packet : public VescFrame {
 public:
  VescStatus3Packet(const struct can_frame &frame);

  float GetWattHours() const { return watt_hours_; }
  float GetWattHoursCharged() const { return watt_hours_charged_; }

 private:
  float watt_hours_;
  float watt_hours_charged_;
};

class VescStatus4Packet : public VescFrame {
 public:
  VescStatus4Packet(const struct can_frame &frame);

  float GetTempFET() const { return temp_fet_; }
  float GetTempMotor() const { return temp_motor_; }
  float GetCurrentIn() const { return current_in_; }
  float GetPidPosNow() const { return pid_pos_now_; }

 private:
  float temp_fet_;
  float temp_motor_;
  float current_in_;
  float pid_pos_now_;
};

class VescStatus5Packet : public VescFrame {
 public:
  VescStatus5Packet(const struct can_frame &frame);

  int32_t GetTachoValue() const { return tacho_value_; }
  float GetVoltageIn() const { return voltage_in_; }

 private:
  int32_t tacho_value_;
  float voltage_in_;
};

class VescStatus6Packet : public VescFrame {
 public:
  VescStatus6Packet(const struct can_frame &frame);

  float GetAdc1() const { return adc_1_; }
  float GetAdc2() const { return adc_2_; }
  float GetAdc3() const { return adc_3_; }
  float GetPpm() const { return ppm_; }

 private:
  float adc_1_;
  float adc_2_;
  float adc_3_;
  float ppm_;
};
}

#endif //ROBOSW_SRC_DRIVER_INCLUDE_VESC_DRIVER_VESC_STATUS_PACKET_HPP
