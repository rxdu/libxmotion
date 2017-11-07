/* 
 * uavcan_subscriber.cpp
 * 
 * Created on: Oct 30, 2017 22:41
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_linux/uavcan_linux.hpp>

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

/*
 * We're going to use messages of type uavcan.protocol.debug.KeyValue, so the appropriate header must be included.
 * Given a data type named X, the header file name would be:
 *      X.replace('.', '/') + ".hpp"
 */
#include <uavcan/protocol/debug/KeyValue.hpp> // uavcan.protocol.debug.KeyValue
#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include <uavcantypes/pixcar/CarRawIMU.hpp>
#include <uavcantypes/pixcar/CarRawSpeed.hpp>

extern uavcan::ICanDriver &getCanDriver();
extern uavcan::ISystemClock &getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

static Node &getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

int main(int argc, const char **argv)
{
    const int self_node_id = 2;

    auto &node = getNode();
    node.setNodeID(self_node_id);
    node.setName("org.uavcan.tutorial.subscriber");

    /*
     * Dependent objects (e.g. publishers, subscribers, servers, callers, timers, ...) can be initialized only
     * if the node is running. Note that all dependent objects always keep a reference to the node object.
     */
    const int node_start_res = node.start();
    if (node_start_res < 0)
    {
        throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
    }

    uavcan::Subscriber<uavcantypes::pixcar::CarRawIMU> imu_sub(node);
    const int imu_sub_start_res =
        imu_sub.start([&](const uavcantypes::pixcar::CarRawIMU &msg) 
        { 
            std::cout << "Gyro: " << msg.gyro[0] << " , " << msg.gyro[1] << " , " << msg.gyro[2] << std::endl; 
            std::cout << "Accel: " << msg.accel[0] << " , " << msg.accel[1] << " , " << msg.accel[2] << std::endl; 
        }
        );

    if (imu_sub_start_res < 0)
    {
        throw std::runtime_error("Failed to start the IMU subscriber; error: " + std::to_string(imu_sub_start_res));
    }

    uavcan::Subscriber<uavcantypes::pixcar::CarRawSpeed> spd_sub(node);
    const int spd_sub_start_res =
        spd_sub.start([&](const uavcantypes::pixcar::CarRawSpeed &msg) 
        { 
            std::cout << "Speed: " << msg.speed << std::endl; 
        }
        );

    if (spd_sub_start_res < 0)
    {
        throw std::runtime_error("Failed to start the Speed subscriber; error: " + std::to_string(spd_sub_start_res));
    }

    /*
     * Running the node.
     */
    node.setModeOperational();

    while (true)
    {
        /*
         * Spinning for 1 second.
         * The method spin() may return earlier if an error occurs (e.g. driver failure).
         * All error codes are listed in the header uavcan/error.hpp.
         */
        const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (spin_res < 0)
        {
            std::cerr << "Transient failure: " << spin_res << std::endl;
        }

        // std::cout << "UAVCAN spinned" << std::endl;
    }
}