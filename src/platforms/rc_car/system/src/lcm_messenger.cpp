/* 
 * lcm_messenger.cpp
 * 
 * Created on: Oct 31, 2017 11:44
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "system/lcm_messenger.hpp"
#include "params/lcm_channels.hpp"

using namespace librav;

LCMMessenger::LCMMessenger(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm)
{
}

void LCMMessenger::republishRawIMUData(const IMU9DOFData &msg)
{
    librav_lcm_msgs::CarRawIMU_t imu_msg;

    imu_msg.mtime = 0;

    imu_msg.gyro[0] = msg.gyro.x;
    imu_msg.gyro[1] = msg.gyro.y;
    imu_msg.gyro[2] = msg.gyro.z;

    imu_msg.accel[0] = msg.accel.x;
    imu_msg.accel[1] = msg.accel.y;
    imu_msg.accel[2] = msg.accel.z;

    imu_msg.magn[0] = msg.magn.x;
    imu_msg.magn[1] = msg.magn.y;
    imu_msg.magn[2] = msg.magn.z;

    lcm_->publish(LCM_CHANNELS::CAR_RAW_IMU_CHANNEL, &imu_msg);
}

// void LCMMessenger::republishRawMagData(const pixcar::CarRawMag &msg)
// {
//     librav_lcm_msgs::CarRawMag_t mag_msg;

//     mag_msg.mtime = msg.time_stamp;

//     mag_msg.mag[0] = msg.mag[0];
//     mag_msg.mag[1] = msg.mag[1];
//     mag_msg.mag[2] = msg.mag[2];

//     lcm_->publish(LCM_CHANNELS::CAR_RAW_MAG_CHANNEL, &mag_msg);
// }

void LCMMessenger::republishRawSpeedData(const Speed &msg)
{
    librav_lcm_msgs::CarRawSpeed_t spd_msg;

    spd_msg.mtime = msg.mtime;
    spd_msg.speed = msg.speed;

    lcm_->publish(LCM_CHANNELS::CAR_RAW_SPEED_CHANNEL, &spd_msg);
}

// void LCMMessenger::publishCalibratedIMUData(const AccGyroData &imu)
// {
//      librav_lcm_msgs::CarRawIMU_t imu_msg;

//     imu_msg.mtime = imu.mtime;

//     imu_msg.gyro[0] = imu.gyro.x;
//     imu_msg.gyro[1] = imu.gyro.y;
//     imu_msg.gyro[2] = imu.gyro.z;

//     imu_msg.accel[0] = imu.accel.x;
//     imu_msg.accel[1] = imu.accel.y;
//     imu_msg.accel[2] = imu.accel.z;

//     lcm_->publish(LCM_CHANNELS::CAR_CALIB_IMU_CHANNEL, &imu_msg);
// }

void LCMMessenger::publishConvertedSpeedData(const Speed &spd)
{
    librav_lcm_msgs::CarRawSpeed_t spd_msg;

    spd_msg.mtime = spd.mtime;
    spd_msg.speed = spd.speed;

    lcm_->publish(LCM_CHANNELS::CAR_COVT_SPEED_CHANNEL, &spd_msg);
}