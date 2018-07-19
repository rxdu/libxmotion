/* 
 * car_control.cpp
 * 
 * Created on: Oct 30, 2017 22:40
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 * 
 * Description: this node manages the communication between the PC/SBC and the MCU on car.
 *      MCU -> PC/SBC: a callback function will be called, receive new sensor data, update control
 *      PC/SBC -> MCU: send control commands of servo and motor
 * 
 *      (real-time performance of this node is desired)
 *  
 */

#include <functional>

#include "comm/can_messenger.h"
#include "comm/lcm_messenger.h"
#include "comm/lcm_channels.h"

#include "sensor/imu_filter.h"

#include "utility/librav_utility.h"

#define WHEEL_DIAMETER 0.065
#define GEAR_RATIO 6.58 // with 20T pinion gear

// #define ENABLE_CSV_LOGGING

using namespace librav;

class CarCommCoordinator
{
  public:
    CarCommCoordinator(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm),
                                                        lcm_messenger_(lcm)
#ifdef ENABLE_CSV_LOGGING
                                                        ,
                                                        imu_logger_(new CsvLogger("raw_imu", "/home/rdu/CarLog")),
                                                        mag_logger_(new CsvLogger("raw_mag", "/home/rdu/CarLog")),
                                                        spd_logger_(new CsvLogger("raw_spd", "/home/rdu/CarLog"))
#endif
    {
        lcm_->subscribe(LCM_CHANNELS::CAR_COMMOND_CHANNEL, &CarCommCoordinator::handleLCMCarCmdMessage, this);
    };

    bool initCarComm()
    {
        if (!setupCallbacks())
            return false;

        return can_messenger_.setCANOperational();
    }

    void startCarComm()
    {
        // spin at ~1kHz
        while (true)
        {
            can_messenger_.spin(1);
            lcm_->handleTimeout(0);
        }
    }

private:
    // communication
    std::shared_ptr<lcm::LCM> lcm_;
    LCMMessenger lcm_messenger_;
    CANMessenger can_messenger_;

    // sensors
    IMUFilter imu_filter_;

#ifdef ENABLE_CSV_LOGGING
    std::unique_ptr<CsvLogger> imu_logger_;
    std::unique_ptr<CsvLogger> mag_logger_;
    std::unique_ptr<CsvLogger> spd_logger_;
#endif

    void uavcanIMUMsgCallback(const pixcar::CarRawIMU &msg)
    {
        // std::cout << "Gyro: " << msg.gyro[0] << " , " << msg.gyro[1] << " , " << msg.gyro[2] << std::endl;
        // std::cout << "Accel: " << msg.accel[0] << " , " << msg.accel[1] << " , " << msg.accel[2] << std::endl;
#ifdef ENABLE_CSV_LOGGING
        imu_logger_->LogData(msg.time_stamp, msg.accel[0], msg.accel[1], msg.accel[2], msg.gyro[0], msg.gyro[1], msg.gyro[2]);
#endif
        
        lcm_messenger_.republishRawIMUData(msg);

        auto corrected_imu_data = 
            imu_filter_.CorrectIMURawData(AccGyroData(msg.time_stamp, msg.accel[0], msg.accel[1], msg.accel[2], 
                                msg.gyro[0], msg.gyro[1], msg.gyro[2]));
        lcm_messenger_.publishCalibratedIMUData(corrected_imu_data);
    }

    void uavcanMagMsgCallback(const pixcar::CarRawMag &msg)
    {
        // std::cout << "Mag: " << msg.mag[0] << " , " << msg.mag[1] << " , " << msg.mag[2] << std::endl;
#ifdef ENABLE_CSV_LOGGING
        mag_logger_->LogData(msg.time_stamp, msg.mag[0], msg.mag[1], msg.mag[2]);
#endif
        lcm_messenger_.republishRawMagData(msg);
    }

    void uavcanSpeedMsgCallback(const pixcar::CarRawSpeed &msg)
    {
        // std::cout << "Speed: " << msg.speed << std::endl;
        float covt_speed = 1.0e6 / (msg.speed * 6.0) / GEAR_RATIO * (M_PI * WHEEL_DIAMETER);
#ifdef ENABLE_CSV_LOGGING
        spd_logger_->LogData(msg.time_stamp, msg.speed, covt_speed);
#endif
        lcm_messenger_.republishRawSpeedData(msg);

        CarSpeed car_speed(0, msg.speed);
        lcm_messenger_.publishConvertedSpeedData(car_speed);
    }

    bool setupCallbacks()
    {
        bool imu_init = can_messenger_.setupIMUSubscriber(std::bind(&CarCommCoordinator::uavcanIMUMsgCallback, this, std::placeholders::_1));
        bool mag_init = can_messenger_.setupMagSubscriber(std::bind(&CarCommCoordinator::uavcanMagMsgCallback, this, std::placeholders::_1));
        bool spd_init = can_messenger_.setupSpeedSubscriber(std::bind(&CarCommCoordinator::uavcanSpeedMsgCallback, this, std::placeholders::_1));

        return (imu_init && mag_init && spd_init);
    }

    void handleLCMCarCmdMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::CarCommand_t *msg)
    {
        std::cout << "received command: " << msg->servo << " , " << msg->motor << std::endl;
        can_messenger_.sendCmdToCar(msg->servo, msg->motor);
    }
};

int main(int argc, char *argv[])
{
    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

    if (!lcm->good())
    {
        std::cout << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

    CarCommCoordinator coordinator(lcm);

    if (!coordinator.initCarComm())
        return -1;

    coordinator.startCarComm();

    return 0;
}