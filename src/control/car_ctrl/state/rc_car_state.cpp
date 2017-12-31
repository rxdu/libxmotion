/*
 * robot_state.cpp
 *
 *  Created on: Jul 29, 2015
 *      Author: rdu
 */

#include <cmath>
#include <iostream>

#include "common/librav_types.hpp"
#include "control/rc_car_ctrl/state/rc_car_state.h"

using namespace librav;

RCCarState::RCCarState(void):
		driving_velocity_(0)
{
}

RCCarState::~RCCarState(void)
{

}

void RCCarState::UpdateRobotState(const RCCarSensorData &new_data)
{
	driving_velocity_ = new_data.body_vel;

	// for(int i = 0; i < IMG_RES_Y; i++)
	// 	for(int j = 0; j < IMG_RES_X; j++)
	// 		mono_image_[i][j] = new_data.mono_image[i][j];
}
