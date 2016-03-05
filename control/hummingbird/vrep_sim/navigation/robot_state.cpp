/*
 * robot_state.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: rdu
 */

#include <cmath>
#include <iostream>
#include <iomanip>
#include "robot_state.h"

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

//	if(last_orientation_.x * orientation_.x < 0)
//	{
//		if(last_orientation_.x < 0)
//			orientation_.x = last_orientation_.x - std::abs(M_PI - std::abs(orientation_.x));
//		else
//			orientation_.x = last_orientation_.x + std::abs(M_PI - std::abs(orientation_.x));
//	}
//
//	if(last_orientation_.y * orientation_.y < 0)
//	{
//		if(last_orientation_.y < 0)
//			orientation_.y = last_orientation_.y - std::abs(M_PI - std::abs(orientation_.y));
//		else
//			orientation_.y = last_orientation_.y + std::abs(M_PI - std::abs(orientation_.y));
//	}
//
//	if(last_orientation_.z * orientation_.z < 0)
//	{
//		if(last_orientation_.z < 0)
//			orientation_.z = last_orientation_.z - std::abs(M_PI - std::abs(orientation_.z));
//		else
//			orientation_.z= last_orientation_.z + std::abs( - std::abs(orientation_.z));
//	}
//

	// quaternion
	Eigen::Quaterniond rotx(Eigen::AngleAxisd(Eigen::AngleAxisd(orientation_.x, Eigen::Vector3d::UnitX())));
	Eigen::Quaterniond roty(Eigen::AngleAxisd(orientation_.y, rotx.matrix().col(1)));
	Eigen::Quaterniond rotz(Eigen::AngleAxisd(orientation_.z, roty.matrix().col(2)));

	quat_ =  rotz * roty * rotx;

	unsigned int euler_inverted_count = 0;
	if((last_orientation_.x * orientation_.x) < 0)
	{
		if(std::abs(last_orientation_.x - orientation_.x) > max_euler_change_) {
			euler_inverted_count++;
			std::cout<< " ******************** euler jump detected ************************"<<std::endl;
		}
	}

	if((last_orientation_.y * orientation_.y) < 0)
	{
		if(std::abs(last_orientation_.y - orientation_.y) > max_euler_change_) {
			euler_inverted_count++;
			std::cout<< " ******************** euler jump detected ************************"<<std::endl;
		}
	}

	if((last_orientation_.z * orientation_.z) < 0)
	{
		if(std::abs(last_orientation_.z - orientation_.z) > max_euler_change_) {
			euler_inverted_count++;
			std::cout<< " ******************** euler jump detected ************************"<<std::endl;
		}
	}

	if(euler_inverted_count % 2 != 0)
		invert_quat = (invert_quat++)%2;

//	if(invert_quat != 0) {
//		quat_.x() =  - quat_.x();
//		quat_.y() =  - quat_.y();
//		quat_.z() =  - quat_.z();
//		quat_.w() =  - quat_.w();
//	}

	last_orientation_.x = orientation_.x;
	last_orientation_.y = orientation_.y;
	last_orientation_.z = orientation_.z;

	last_quat_ = quat_;
//	std::cout<<"robot orientation (w,x,y,z): "<< std::setw(15) << quat_.w()<<" , "<< std::setw(15) <<quat_.x()<<" , "<< std::setw(15) <<quat_.y()<<" , "<< std::setw(15) << quat_.z()<<std::endl;

//	std::cout<<"euler: "
//					<< std::setw(12) << orientation_.x
//					<< " , " << std::setw(12) << orientation_.y
//					<< " , " << std::setw(12) << orientation_.z
//					<< std::setw(15) << " quaternion: "
//					<< std::setw(12) << quat_.w()
//					<< " , " << std::setw(12) << quat_.x()
//					<< " , " << std::setw(12) << quat_.y()
//					<< " , " << std::setw(12) << quat_.z()
//					<< " * " << invert_quat
//					<<std::endl;

	rotation_rate_.x = new_data.rot_rate_b.x;
	rotation_rate_.y = new_data.rot_rate_b.y;
	rotation_rate_.z = new_data.rot_rate_b.z;
}


