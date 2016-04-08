/*
 * robot_state.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#include <cmath>
#include <iostream>
#include <iomanip>

#include "g3log/g3log.hpp"

#include "estimator/robot_state.h"
#include "utils/utils_log.h"

using namespace srcl_ctrl;

RobotState::RobotState():
		g_(9.8),
		max_euler_change_(3.14),
		mass_(0.57375),
		arm_length_(0.175),
		kF_(6.11e-8),
		kM_(1.5e-9),
		sim_step_(0.01),
		invert_quat(false)
{
	w_h_ = sqrt(mass_ * g_ / 4 / kF_);

	// get values directly from simulator, will be replaced by state estimator later
	position_.x = 0;
	position_.y = 0;
	position_.z = 0.04;

	velocity_.x = 0;
	velocity_.y = 0;
	velocity_.z = 0;

	// euler in X-Y-Z convension
	orientation_.x = 0;
	orientation_.y = 0;
	orientation_.z = 0;

	quat_.x() = 0;
	quat_.y() = 0;
	quat_.z() = 0;
	quat_.w() = 0;

	last_orientation_.x = 0;
	last_orientation_.y = 0;
	last_orientation_.z = 0;

	rotation_rate_.x = 0;
	rotation_rate_.y = 0;
	rotation_rate_.z = 0;
}

RobotState::~RobotState()
{

}

void RobotState::UpdateRobotState(const DataFromRobot & new_data)
{
	// get values directly from simulator, will be replaced by state estimator later
	position_.x = new_data.pos_i.x;
	position_.y = new_data.pos_i.y;
	position_.z = new_data.pos_i.z;

	velocity_.x = new_data.vel_i.x;
	velocity_.y = new_data.vel_i.y;
	velocity_.z = new_data.vel_i.z;

	// euler in X-Y-Z convension
	orientation_.x = new_data.rot_i.x;
	orientation_.y = new_data.rot_i.y;
	orientation_.z = new_data.rot_i.z;

	// quaternion
	quat_.x() = new_data.quat_i.x;
	quat_.y() = new_data.quat_i.y;
	quat_.z() = new_data.quat_i.z;
	quat_.w() = new_data.quat_i.w;

	rotation_rate_.x = new_data.rot_rate_b.x;
	rotation_rate_.y = new_data.rot_rate_b.y;
	rotation_rate_.z = new_data.rot_rate_b.z;

#ifdef ENABLE_LOG
	UtilsLog::AppendLogMsgTuple3f(position_.x,position_.y,position_.z);
	UtilsLog::AppendLogMsgTuple3f(velocity_.x,velocity_.y,velocity_.z);
	UtilsLog::AppendLogMsgTuple3f(orientation_.x,orientation_.y,orientation_.z);
	UtilsLog::AppendLogMsgTuple4f(quat_.w(), quat_.x(), quat_.y(), quat_.z());
	UtilsLog::AppendLogMsgTuple3f(rotation_rate_.x,rotation_rate_.y,rotation_rate_.z);
#endif
}
