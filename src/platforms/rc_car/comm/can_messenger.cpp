/* 
 * can_messenger.cpp
 * 
 * Created on: Oct 30, 2017 22:40
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include <iostream>
#include "comm/can_messenger.h"

using namespace librav;

namespace
{
uavcan::ISystemClock &getSystemClock()
{
    static uavcan_linux::SystemClock clock;
    return clock;
}

uavcan::ICanDriver &getCanDriver()
{
    static uavcan_linux::SocketCanDriver driver(dynamic_cast<const uavcan_linux::SystemClock &>(getSystemClock()));
    if (driver.getNumIfaces() == 0) // Will be executed once
    {
        if (driver.addIface("slcan0") < 0)
        {
            throw std::runtime_error("Failed to add iface");
        }
    }
    return driver;
}
}

CANMessenger::CANMessenger():
    node_(CANMessenger::getNode()),
    cmd_pub_(node_),
    imu_sub_(node_),
    mag_sub_(node_),
    spd_sub_(node_),
    imu_sub_init_(false),
    mag_sub_init_(false),
    spd_sub_init_(false),
    running_(false)
    //,logger_(new CsvLogger("raw_imu", "/home/rdu/Workspace/auto_racing/data/log"))
{
    // init CAN node
    node_.setNodeID(CAN_MESSENGER_NODE_ID);
    node_.setName("pixcar.can_messenger");

    auto node_start_res = node_.start();
    if (node_start_res < 0)
    {
        throw std::runtime_error("Failed to start the CAN node; error: " + std::to_string(node_start_res));
    }
    
    // init publishers
    auto cmd_pub_init_res = cmd_pub_.init();
    if (cmd_pub_init_res < 0)
    {
        throw std::runtime_error("Failed to start the CMD publisher; error: " + std::to_string(cmd_pub_init_res));
    }
    cmd_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(5));
    cmd_pub_.setPriority(uavcan::TransferPriority(1));  
}

UAVCANNode& CANMessenger::getNode()
{
    static UAVCANNode node(getCanDriver(), getSystemClock());
    return node;
}

bool CANMessenger::setupIMUSubscriber(std::function<void (const pixcar::CarRawIMU &msg)> callback)
{
    auto imu_sub_start_res = imu_sub_.start(callback);

    if (imu_sub_start_res < 0)
    {
        throw std::runtime_error("Failed to start the IMU subscriber; error: " + std::to_string(imu_sub_start_res));
    }

    imu_sub_init_ = (imu_sub_start_res < 0? false:true);

    return imu_sub_init_; 
}

bool CANMessenger::setupMagSubscriber(std::function<void (const pixcar::CarRawMag &msg)> callback)
{
    auto mag_sub_start_res = mag_sub_.start(callback);
    
    if (mag_sub_start_res < 0)
    {
        throw std::runtime_error("Failed to start the MAG subscriber; error: " + std::to_string(mag_sub_start_res));
    }
    
    mag_sub_init_ = (mag_sub_start_res < 0? false:true);
    
    return mag_sub_init_; 
}

bool CANMessenger::setupSpeedSubscriber(std::function<void (const pixcar::CarRawSpeed &msg)> callback)
{
    auto spd_sub_start_res =
    spd_sub_.start(callback);

    if (spd_sub_start_res < 0)
    {
        throw std::runtime_error("Failed to start the Speed subscriber; error: " + std::to_string(spd_sub_start_res));
    }

    spd_sub_init_ = (spd_sub_start_res < 0? false:true);

    return spd_sub_init_; 
}

bool CANMessenger::setCANOperational()
{
    // set node operational
    if(imu_sub_init_ && spd_sub_init_)
    {
        node_.setModeOperational();
        running_ = true;
    }
    else
    {
        std::cerr << "Subscriber init status: IMU - " << imu_sub_init_ << " , Speed - " << spd_sub_init_ << std::endl;
        running_ = false;
    }
    
    return running_;
}

void CANMessenger::spin(int32_t timeout_ms)
{
    auto spin_res = node_.spin(uavcan::MonotonicDuration::fromMSec(timeout_ms));
    if (spin_res < 0)
    {
        std::cerr << "Transient failure: " << spin_res << std::endl;
    }
}

bool CANMessenger::sendCmdToCar(int8_t servo, int8_t motor)
{
    pixcar::CarCommand cmd_msg; 

    std::cout << "cmd signature : " << cmd_msg.getDataTypeSignature() << std::endl;

    cmd_msg.servo_cmd = (int8_t)servo;
    cmd_msg.motor_cmd = (int8_t)motor;

    auto cmd_pub_res = cmd_pub_.broadcast(cmd_msg);
    if (cmd_pub_res < 0)
    {
        std::cerr << "Command publication failure: " << cmd_pub_res << std::endl;
    }

    return (cmd_pub_res < 0? false:true);
}