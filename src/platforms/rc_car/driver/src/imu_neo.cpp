/* 
 * imu_neo.cpp
 * 
 * Created on: Jan 13, 2018 15:56
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "driver/imu_neo.hpp"
#include "system/car_params.hpp"

using namespace librav;

IMUNeo::IMUNeo() : fxos8700_(new FXOS8700CQ(3)),
                   fxas21002_(new FXAS21002C(3))
{
    // use parameters for Tamiya TA07 Pro model
    accel_calib_ = TamiyaTA07ProParams::GetParams().accel_calib;
    gyro_calib_ = TamiyaTA07ProParams::GetParams().accel_calib;
}

bool IMUNeo::InitIMU()
{
    if (fxos8700_->ConfigI2CBus())
    {
        std::cout << "FXOS8700CQ Sensor connected" << std::endl;
    }
    else
    {
        std::cerr << "FXOS8700CQ Sensor not responding" << std::endl;
        return false;
    }

    if (fxos8700_->ConfigSensor())
    {
        std::cout << "FXOS8700CQ Sensor initialized" << std::endl;
    }
    else
    {
        std::cerr << "FXOS8700CQ Sensor failed to initialize" << std::endl;
        return false;
    }

    if (fxas21002_->ConfigI2CBus())
    {
        std::cout << "FXAS21002C Sensor connected" << std::endl;
    }
    else
    {
        std::cerr << "FXAS21002C Sensor not responding" << std::endl;
        return false;
    }

    if (fxas21002_->ConfigSensor())
    {
        std::cout << "FXAS21002C Sensor initialized" << std::endl;
    }
    else
    {
        std::cerr << "FXAS21002C Sensor failed to initialize" << std::endl;
        return false;
    }

    // get scale to calculate physical values from raw values
    accel_scale_ = fxos8700_->GetAccelScale();
    magn_scale_ = fxos8700_->GetMagnScale();
    gyro_scale_ = fxas21002_->GetGyroScale();

    // set flag to be ready
    imu_ready_ = true;

    return true;
}

bool IMUNeo::GetIMUData(IMU9DOFData *imu_data)
{
    if (!imu_ready_)
    {
        std::cerr << "IMU sensor not initialized properly" << std::endl;
        return false;
    }

    // get data from FXOS8700
    if (!fxos8700_->ReadAccelMagnData(&accel_data_, &magn_data_))
    {
        std::cerr << "failed to read fxos8700" << std::endl;
        return false;
    }
    // get data from FXAS21002
    if (!fxas21002_->ReadGyroData(&gyro_data_))
    {
        std::cerr << "failed to read fxas21002" << std::endl;
        return false;
    }

    // std::cout << "accel: " << accel_data_.x << " , " << accel_data_.y << " , " << accel_data_.z << " ; "
    //           << "mag: " << magn_data_.x << " , " << magn_data_.y << " , " << magn_data_.z << " ; "
    //           << "gyro: " << gyro_data_.x << " , " << gyro_data_.y << " , " << gyro_data_.z << std::endl;

    imu_data->accel.x = accel_data_.x * accel_scale_;
    imu_data->accel.y = accel_data_.y * accel_scale_;
    imu_data->accel.z = accel_data_.z * accel_scale_;

    imu_data->magn.x = magn_data_.x * magn_scale_;
    imu_data->magn.y = magn_data_.y * magn_scale_;
    imu_data->magn.z = magn_data_.z * magn_scale_;

    imu_data->gyro.x = gyro_data_.x * gyro_scale_;
    imu_data->gyro.y = gyro_data_.y * gyro_scale_;
    imu_data->gyro.z = gyro_data_.z * gyro_scale_;

    return true;
}

IMU6DOFData IMUNeo::CorrectIMURawData(const IMU6DOFData &raw)
{
    Eigen::Matrix<double, 3, 1> raw_acc, raw_gyro;
    raw_acc << raw.accel.x, raw.accel.y, raw.accel.z;
    raw_gyro << raw.gyro.x, raw.gyro.y, raw.gyro.z;

    Eigen::Matrix<double, 3, 1> cor_acc, cor_gyro;

    cor_acc = accel_calib_.misalignment_matrix * accel_calib_.scale_matrix * (raw_acc + accel_calib_.bias_vector);
    cor_gyro = gyro_calib_.misalignment_matrix * gyro_calib_.scale_matrix * (raw_gyro + gyro_calib_.bias_vector);

    return IMU6DOFData(raw.mtime, cor_acc(0), cor_acc(1), cor_acc(2), cor_gyro(0), cor_gyro(1), cor_gyro(2));
}