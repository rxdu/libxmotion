/* 
 * can_messenger.cpp
 * 
 * Created on: Oct 30, 2017 22:40
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include <iostream>
#include "system/can_messenger.hpp"

using namespace librav;

CANMessenger::CANMessenger(std::string can_iface_name)
//:logger_(new CsvLogger("raw_imu", "/home/rdu/Workspace/auto_racing/data/log"))
{
    SocketCANInstance socketcan;
    std::string ifacename = "can0";
    int res = socketcanInit(&socketcan, ifacename.c_str());
    socketcan_inst_ = socketcan;
    if (res < 0)
    {
        std::cerr << "Failed to open CAN interface " << can_iface_name_ << std::endl;
    }
    else
    {
        std::cout << "CAN interface " << can_iface_name_ << " initialized successfully , fd: " << socketcan_inst_.fd << std::endl;
        ready_ = true;
    }
}

CANMessenger::~CANMessenger()
{
    socketcanClose(&socketcan_inst_);
}

void CANMessenger::SetupSpeedSubscriber(std::function<void(const Speed &spd_msg)> callback)
{
    spd_callback_ = callback;
}

void CANMessenger::handleTimeout(int32_t timeout_ms)
{
    // Receiving
    CanardCANFrame rx_frame;
    const int rx_res = socketcanReceive(&socketcan_inst_, &rx_frame, timeout_ms);
    if (rx_res < 0) // Failure - report
    {
        std::cerr << "Receive error " << rx_res << ", frame dropped " << std::endl;
        return;
    }

    // We only use extended ID in this system
    rx_frame.id &= ~0x80000000U;
    ProcessCANFrame(rx_frame);
}

void CANMessenger::ProcessCANFrame(const CanardCANFrame &frame)
{
    static Speed spd_msg;
    uint32_t spd_temp = 0;

    switch (frame.id)
    {
    case CANTALK_AUTOCAR_MCUHEARTBEAT_DATA_TYPE_ID:
        // std::cout << "MCU heartbeat frame received" << std::endl;
        break;
    case CANTALK_AUTOCAR_CARSPEED_DATA_TYPE_ID:
        // std::cout << "Car speed frame received" << std::endl;
        spd_temp = (uint32_t(frame.data[3]) << 24) | (uint32_t(frame.data[2]) << 16) | (uint32_t(frame.data[1]) << 8) | uint32_t(frame.data[0]);
        spd_msg.mtime = 0;
        // spd_msg.speed = static_cast<float>(spd_temp);
        memcpy(&spd_msg.speed, &spd_temp, sizeof(spd_temp));
        spd_callback_(spd_msg);
        break;
        // default:
        // std::cout << "Unkown CAN frame with id: " << frame.id << std::endl;
    }
}

bool CANMessenger::SendCmdToCar(bool update_servo, float servo, bool update_motor, float motor, int timeout_ms)
{
    if (!ready_)
    {
        std::cerr << "CAN interface not initialized properly" << std::endl;
        return false;
    }
    // return directly if no update needed
    if (!update_servo && !update_motor)
        return true;

    // convert command
    int8_t servo_cmd = static_cast<int8_t>(servo * 100);
    int8_t motor_cmd = static_cast<int8_t>(motor * 100);

    uint8_t cmd_flag = 0;
    if (update_servo)
        cmd_flag |= 0x01;
    if (update_motor)
        cmd_flag |= 0x01 << 1;

    // Transmitting
    CanardCANFrame tx_frame;
    // use extended ID
    tx_frame.id = CANTALK_AUTOCAR_CARCOMMAND_DATA_TYPE_ID | 0x80000000U;
    tx_frame.data[0] = cmd_flag;
    memcpy(&tx_frame.data[1], &servo_cmd, sizeof(uint8_t));
    memcpy(&tx_frame.data[2], &motor_cmd, sizeof(uint8_t));
    tx_frame.data_len = 3;
    const int tx_res = socketcanTransmit(&socketcan_inst_, &tx_frame, timeout_ms);

    bool tx_result = false;
    if (tx_res < 0) // Failure - drop the frame and report
    {
        std::cout << "Transmit error " << tx_res << ", frame dropped" << std::endl;
    }
    else if (tx_res > 0) // Success - just drop the frame
    {
        std::cout << "Transmit successfully" << std::endl;
        tx_result = true;
    }
    // else
    // Timeout - just exit and try again later

    return true;
}

bool CANMessenger::SendActuatorCmdToCar(float servo, float motor, int timeout_ms)
{
    return SendCmdToCar(true, servo, true, motor, timeout_ms);
}

bool CANMessenger::SendServoCmdToCar(float servo, int timeout_ms)
{
    return SendCmdToCar(true, servo, false, 0, timeout_ms);
}

bool CANMessenger::SendMotorCmdToCar(float motor, int timeout_ms)
{
    return SendCmdToCar(false, 0, true, motor, timeout_ms);
}
