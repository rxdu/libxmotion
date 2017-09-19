/*
 * sim_depth_sensor.h
 *
 *  Created on: Sep 19, 2017
 *      Author: rdu
 */

#ifndef QUADROTOR_PATH_REPAIR_SIM_DEPTH_SENSOR_H
#define QUADROTOR_PATH_REPAIR_SIM_DEPTH_SENSOR_H

#include <cstdint>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

namespace librav
{

class SimDepthSensor
{
public:
    SimDepthSensor();

    void SetWorkspace(const librav_lcm_msgs::Map_t *msg);

private:
    int32_t ws_x_;
    int32_t ws_y_;
    int32_t ws_z_;

};

}

#endif /* QUADROTOR_PATH_REPAIR_SIM_DEPTH_SENSOR_H */