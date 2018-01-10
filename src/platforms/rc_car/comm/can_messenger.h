/* 
 * can_messenger.h
 * 
 * Created on: Oct 30, 2017 22:40
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef CAN_MESSENGER_H
#define CAN_MESSENGER_H

#include <cstdint>
#include <memory>
#include <functional>

#include <uavcan/uavcan.hpp>
#include <uavcan_linux/uavcan_linux.hpp>

#include "utility/logging/logger.hpp"

// UAVCAN types
#include <pixcar/CarRawIMU.hpp>
#include <pixcar/CarRawMag.hpp>
#include <pixcar/CarRawSpeed.hpp>
#include <pixcar/CarCommand.hpp>

namespace librav
{

constexpr uint32_t NodeMemoryPoolSize = 65536;
typedef uavcan::Node<NodeMemoryPoolSize> UAVCANNode;
constexpr int16_t CAN_MESSENGER_NODE_ID = 2;

class CANMessenger
{
public:
    CANMessenger();
    ~CANMessenger() = default;

    // setup functions
    bool setupIMUSubscriber(std::function<void (const pixcar::CarRawIMU &msg)> callback);
    bool setupMagSubscriber(std::function<void (const pixcar::CarRawMag &msg)> callback);
    bool setupSpeedSubscriber(std::function<void (const pixcar::CarRawSpeed &msg)> callback);
    bool setCANOperational();

    // this function should be called periodically
    void spin(int32_t timeout_ms);
    
    bool sendCmdToCar(int8_t servo, int8_t motor);

private:
    // the only CAN node that talks with car
    UAVCANNode& node_;
        
    // publishers
    uavcan::Publisher<pixcar::CarCommand> cmd_pub_;

    // subscribers
    uavcan::Subscriber<pixcar::CarRawIMU> imu_sub_;
    uavcan::Subscriber<pixcar::CarRawMag> mag_sub_;
    uavcan::Subscriber<pixcar::CarRawSpeed> spd_sub_;

    // messenger state flags
    bool imu_sub_init_;
    bool mag_sub_init_;
    bool spd_sub_init_;
    bool running_;

    std::unique_ptr<CsvLogger> logger_;

    static UAVCANNode &getNode();
};
}

#endif /* CAN_MESSENGER_H */