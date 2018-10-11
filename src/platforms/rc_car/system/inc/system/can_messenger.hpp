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

#include "common/librav_types.hpp"
#include "socketcan/socketcan.h"

#include "logging/logger.hpp"

namespace librav
{

// Smaller ID -> Higher priority
#define CANTALK_AUTOCAR_MCUHEARTBEAT_DATA_TYPE_ID    3301
#define CANTALK_AUTOCAR_SBCHEARTBEAT_DATA_TYPE_ID    3302

#define CANTALK_AUTOCAR_CARCOMMAND_DATA_TYPE_ID      3311
#define CANTALK_AUTOCAR_CARSPEED_DATA_TYPE_ID        3312

class CANMessenger
{
  public:
    CANMessenger(std::string can_iface_name = "can0");
    ~CANMessenger();

    // setup functions
    void SetupSpeedSubscriber(std::function<void (const CarSpeed &spd_msg)> callback);

    // this function should be called periodically
    void handleTimeout(int32_t timeout_ms);
   
    bool SendActuatorCmdToCar(float servo, float motor, int timeout_ms);
    bool SendServoCmdToCar(float servo, int timeout_ms);
    bool SendMotorCmdToCar(float motor, int timeout_ms);

  private:
    bool ready_ = false;

    SocketCANInstance socketcan_inst_;
    std::string can_iface_name_;

    std::unique_ptr<CsvLogger> logger_;

    std::function<void (const CarSpeed &msg)> spd_callback_;

    void ProcessCANFrame(const CanardCANFrame& frame);
    bool SendCmdToCar(bool update_servo, float servo, bool update_motor, float motor, int timeout_ms);
};
}

#endif /* CAN_MESSENGER_H */