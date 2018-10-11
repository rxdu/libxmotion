/* 
 * imu_neo.hpp
 * 
 * Created on: Jan 13, 2018 15:55
 * Description: configuration for IMU on Udoo Neo Full
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef IMU_NEO_HPP
#define IMU_NEO_HPP

#include <memory>

#include "common/librav_types.hpp"
#include "sensors/fxos8700cq.hpp"
#include "sensors/fxas21002c.hpp"

namespace librav
{

class IMUNeo
{
public:
  IMUNeo();
  ~IMUNeo() = default;

  bool InitIMU();
  bool GetIMUData(IMU9DOFData* imu_data);

private:
  std::unique_ptr<FXOS8700CQ> fxos8700_;
  std::unique_ptr<FXAS21002C> fxas21002_;

  bool imu_ready_;

  FXOS8700_RawData accel_data_;
  FXOS8700_RawData magn_data_;
  FXAS21002_RawData gyro_data_;

  double accel_scale_;
  double magn_scale_;
  double gyro_scale_;

  IMUCalibParams accel_calib_;
  IMUCalibParams gyro_calib_;

  IMU6DOFData CorrectIMURawData(const IMU6DOFData &raw);
};
}

#endif /* IMU_NEO_HPP */
