/*
 * sim_depth_sensor.cpp
 *
 *  Created on: Sep 19, 2017
 *      Author: rdu
 */

#include "quadrotor/path_repair/sim/sim_depth_sensor.h"

using namespace librav;

SimDepthSensor::SimDepthSensor() : ws_x_(5),
                                   ws_y_(5), 
                                   ws_z_(5)
{
}

void SimDepthSensor::SetWorkspace(const librav_lcm_msgs::Map_t *msg)
{
   
}