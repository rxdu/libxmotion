/* 
 * nerve_system.cpp
 * 
 * Created on: Jan 13, 2018 14:57
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "system/nerve_system.hpp"

#include <functional>

using namespace librav;

NerveSystem::NerveSystem(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm),
                                                          lcm_messenger_(lcm),
                                                          imu_sensor_(new IMUNeo())
#ifdef ENABLE_CSV_LOGGING
                                                          ,
                                                          imu_logger_(new CsvLogger("raw_imu", "/home/rdu/CarLog")),
                                                          mag_logger_(new CsvLogger("raw_mag", "/home/rdu/CarLog")),
                                                          spd_logger_(new CsvLogger("raw_spd", "/home/rdu/CarLog"))
#endif
                                                              {

                                                              };

bool NerveSystem::init()
{
    // init sensors
    if (imu_sensor_->InitIMU())
        system_health_.imu_sensor = ModuleStatus::NORMAL;
    else
        system_health_.imu_sensor = ModuleStatus::CRITICAL;

    // subscribe to LCM channel
    lcm_->subscribe(LCM_CHANNELS::CAR_COMMOND_CHANNEL, &NerveSystem::SendCarCmdFromLCMMessage, this);

    // setup can bus callbacks
    can_messenger_.SetupSpeedSubscriber(std::bind(&NerveSystem::ReceiveHallSpeedData, this, std::placeholders::_1));

    // default module status, to be updated after system starts
    system_health_.can_bus = ModuleStatus::NORMAL;
    system_health_.lcm_network = ModuleStatus::NORMAL;

    return system_health_.isNormal();
}

void NerveSystem::start()
{
    uint64_t loop_counter = 0;

    // spin at ~500Hz
    while (true)
    {
        stop_watch_.tic();

        // imu sensor poll
        //if(loop_counter%2)
        ReceiveIMUData();

        // can bus poll
        can_messenger_.handleTimeout(0);

        // lcm poll
        lcm_->handleTimeout(0);

        // sleep until the loop reaches 2ms
        stop_watch_.sleep_until_ms(1);

        // increase loop counter
        ++loop_counter;
    }
}

void NerveSystem::ReceiveIMUData()
{
    IMU9DOFData imu_data;
    imu_sensor_->GetIMUData(&imu_data);

    /*std::cout << "accel: " << imu_data.accel.x << " , " << imu_data.accel.y << " , " << imu_data.accel.z << " ; "
              << "mag: " << imu_data.magn.x << " , " << imu_data.magn.y << " , " << imu_data.magn.z << " ; "
              << "gyro: " << imu_data.gyro.x << " , " << imu_data.gyro.y << " , " << imu_data.gyro.z << std::endl; */

#ifdef ENABLE_CSV_LOGGING
    imu_logger_->LogData(msg.time_stamp, msg.accel[0], msg.accel[1], msg.accel[2], msg.gyro[0], msg.gyro[1], msg.gyro[2]);
#endif

    lcm_messenger_.republishRawIMUData(imu_data);

    //     auto corrected_imu_data =
    //         imu_filter_.CorrectIMURawData(AccGyroData(msg.time_stamp, msg.accel[0], msg.accel[1], msg.accel[2],
    //                                                   msg.gyro[0], msg.gyro[1], msg.gyro[2]));
    //     lcm_messenger_.publishCalibratedIMUData(corrected_imu_data);

    //     // std::cout << "Mag: " << msg.mag[0] << " , " << msg.mag[1] << " , " << msg.mag[2] << std::endl;
    // #ifdef ENABLE_CSV_LOGGING
    //     mag_logger_->LogData(msg.time_stamp, msg.mag[0], msg.mag[1], msg.mag[2]);
    // #endif
    //     lcm_messenger_.republishRawMagData(msg);
}

void NerveSystem::ReceiveHallSpeedData(const CarSpeed &spd_msg)
{
    std::cout << "Speed: " << spd_msg.speed << std::endl;

#ifdef ENABLE_CSV_LOGGING
    spd_logger_->LogData(spd_msg.time_stamp, spd_msg.speed);
#endif
    lcm_messenger_.republishRawSpeedData(spd_msg);

    // CarSpeed car_speed(msg.time_stamp, msg.speed_estimate);
    // lcm_messenger_.publishConvertedSpeedData(car_speed);
}

void NerveSystem::SendCarCmdFromLCMMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::CarCommand_t *msg)
{
    std::cout << "received command: " << msg->servo << " , " << msg->motor << std::endl;
    
    if (msg->update_servo && msg->update_motor)
    {
        can_messenger_.SendActuatorCmdToCar(msg->servo, msg->motor, 0);
    }
    else
    {
        if (msg->update_servo)
            can_messenger_.SendServoCmdToCar(msg->servo, 0);
        if (msg->update_motor)
            can_messenger_.SendMotorCmdToCar(msg->motor, 0);
    }
}
