/*
 * robot_state.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#include <cmath>
#include "robot_state.h"

using namespace srcl_ctrl;

RobotState::RobotState():
		g(9.8),mass(0.57375),
		kF(6.11e-8),kM(1.5e-9)
{
	w_h = sqrt(mass * g / 4 / kF);

	// get values directly from simulator, will be replaced by state estimator later
	position.x = 0;
	position.y = 0;
	position.z = 0.04;

	velocity.x = 0;
	velocity.y = 0;
	velocity.z = 0;

	// euler in X-Y-Z convension
	orientation.x = 0;
	orientation.y = 0;
	orientation.z = 0;

	rotation_rate.x = 0;
	rotation_rate.y = 0;
	rotation_rate.z = 0;
}

RobotState::~RobotState()
{

}

void RobotState::UpdateRobotState(const DataFromRobot & new_data)
{
	// get values directly from simulator, will be replaced by state estimator later
	position.x = new_data.pos_i.x;
	position.y = new_data.pos_i.y;
	position.z = new_data.pos_i.z;

	velocity.x = new_data.vel_i.x;
	velocity.y = new_data.vel_i.y;
	velocity.z = new_data.vel_i.z;

	// euler in X-Y-Z convension
	orientation.x = new_data.rot_i.x;
	orientation.y = new_data.rot_i.y;
	orientation.z = new_data.rot_i.z;

	rotation_rate.x = new_data.rot_rate_b.x;
	rotation_rate.y = new_data.rot_rate_b.y;
	rotation_rate.z = new_data.rot_rate_b.z;
}


