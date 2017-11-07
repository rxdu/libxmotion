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

#include "rc_car/comm/can_messenger.h"
#include "rc_car/comm/lcm_messenger.h"

using namespace librav;

class CarCommCoordinator
{
public:
    CarCommCoordinator(std::shared_ptr<lcm::LCM> lcm):
        lcm_(lcm),
        lcm_messenger_(lcm){};

    bool initCarComm()
    {
        if(!setupCallbacks())
            return false;

        return can_messenger_.setCANOperational();
    }

    void startCarComm()
    {
        // spin at 1kHz
        while(true)
        {
            can_messenger_.spin(1);
            lcm_->handleTimeout(0);
        }
    }

private:
    std::shared_ptr<lcm::LCM> lcm_;

    LCMMessenger lcm_messenger_;    
    CANMessenger can_messenger_;

    void uavcanIMUMsgCallback(const uavcantypes::pixcar::CarRawIMU &msg)
    {
        std::cout << "Gyro: " << msg.gyro[0] << " , " << msg.gyro[1] << " , " << msg.gyro[2] << std::endl; 
        std::cout << "Accel: " << msg.accel[0] << " , " << msg.accel[1] << " , " << msg.accel[2] << std::endl; 

        lcm_messenger_.republishRawIMUData(msg);
    }

    void uavcanMagMsgCallback(const uavcantypes::pixcar::CarRawMag &msg)
    {
        std::cout << "Mag: " << msg.mag[0] << " , " << msg.mag[1] << " , " << msg.mag[2] << std::endl; 

        lcm_messenger_.republishRawMagData(msg);
    }
    
    void uavcanSpeedMsgCallback(const uavcantypes::pixcar::CarRawSpeed &msg)
    {
        std::cout << "Speed: " << msg.speed << std::endl;     

        lcm_messenger_.republishRawSpeedData(msg);
    }

    bool setupCallbacks()
    {
        bool imu_init = can_messenger_.setupIMUSubscriber(std::bind(&CarCommCoordinator::uavcanIMUMsgCallback, this, std::placeholders::_1));
        bool mag_init = can_messenger_.setupMagSubscriber(std::bind(&CarCommCoordinator::uavcanMagMsgCallback, this, std::placeholders::_1));
        bool spd_init = can_messenger_.setupSpeedSubscriber(std::bind(&CarCommCoordinator::uavcanSpeedMsgCallback, this, std::placeholders::_1));

        return (imu_init && mag_init && spd_init);
    }
};

int main(int argc, char* argv[])
{
    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
    
    if(!lcm->good())
    {
        std::cout << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

    CarCommCoordinator coordinator(lcm);

    if(!coordinator.initCarComm())
        return -1;

    coordinator.startCarComm();

    return 0;
}