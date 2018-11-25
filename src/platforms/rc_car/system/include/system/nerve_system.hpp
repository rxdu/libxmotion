/* 
 * nerve_system.hpp
 * 
 * Created on: Jan 13, 2018 14:56
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RCCAR_NERVE_SYSTEM_HPP
#define RCCAR_NERVE_SYSTEM_HPP

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "driver/imu_neo.hpp"

#include "system/can_messenger.hpp"
#include "system/lcm_messenger.hpp"
#include "params/lcm_channels.hpp"

#include "logging/logger.hpp"
#include "stopwatch/stopwatch.h"

namespace librav
{

enum class ModuleStatus : int
{
    NORMAL = 0,
    ERROR,
    CRITICAL
};

struct SystemHealth
{
    ModuleStatus imu_sensor;
    ModuleStatus can_bus;
    ModuleStatus lcm_network;

    bool isNormal()
    {
        if (imu_sensor != ModuleStatus::NORMAL)
            return false;
        if (can_bus != ModuleStatus::NORMAL)
            return false;
        if (lcm_network != ModuleStatus::NORMAL)
            return false;
        
        // all check passed
        return true;
    }
};

class NerveSystem
{
  public:
    NerveSystem(std::shared_ptr<lcm::LCM> lcm);

    bool init();
    void start();

  private:
    // system health monitor
    SystemHealth system_health_;

    // communication
    std::shared_ptr<lcm::LCM> lcm_;
    LCMMessenger lcm_messenger_;
    CANMessenger can_messenger_;

    // timing control
    stopwatch::StopWatch stop_watch_;

    // sensors
    std::unique_ptr<IMUNeo> imu_sensor_;

#ifdef ENABLE_CSV_LOGGING
    std::unique_ptr<CsvLogger> imu_logger_;
    std::unique_ptr<CsvLogger> mag_logger_;
    std::unique_ptr<CsvLogger> spd_logger_;
#endif

    // MCU -> NEO
    void ReceiveIMUData();
    void ReceiveHallSpeedData(const Speed & spd_msg);

    // NEO -> MCU
    void SendCarCmdFromLCMMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::CarCommand_t *msg);
};
}

#endif /* RCCAR_NERVE_SYSTEM_HPP */
