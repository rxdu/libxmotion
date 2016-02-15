/*
 * robot_state.cpp
 *
 *  Created on: Jul 29, 2015
 *      Author: rdu
 */

#include <cmath>
#include <iostream>

#include "library/eigen3/Eigen/Core"

#include "robot_state/robot_state.h"

using namespace srcl_ctrl;

RobotState::RobotState(void)
{
	JointState init_joint;

	init_joint.q[0] = 0;
	init_joint.q[1] = 0;
	init_joint.q_dot[0] = 0;
	init_joint.q_dot[1] = 0;

	UpdateRobotState(init_joint);
}

RobotState::~RobotState(void)
{

}

void RobotState::UpdateRobotState(const JointState &joint)
{
//	long q1_int;
//	long q2_int;
//
//	q1_int = ((long)(joint.q[0]*10000))%31400;
//	q2_int = ((long)(joint.q[0]*10000))%31400;
//
//	float q1 = q1_int/10000.0;
//	float q2 = q2_int/10000.0;
	float q1 = joint.q[0];
	float q2 = joint.q[1];
	float q_dot1 = joint.q_dot[0];
	float q_dot2 = joint.q_dot[1];

	M_matrix_(0,0) = m1*lc1*lc1 + m2*(l1*l1 + lc2*lc2 + 2*l1*lc2*std::cos(q2))+I1+I2;
	M_matrix_(0,1) = m2*(lc2*lc2 + l1*lc2*std::cos(q2)) + I2;
	M_matrix_(1,0) = m2*(lc2*lc2 + l1*lc2*std::cos(q2)) + I2;
	M_matrix_(1,1) = m2*lc2*lc2 + I2;

	C_matrix_(0,0) = -m2*l1*lc2*std::sin(q2)*q_dot2;
	C_matrix_(0,0) = -m2*l1*lc2*std::sin(q2)*(q_dot1+q_dot2);
	C_matrix_(0,0) = m2*l1*lc2*std::sin(q2)*q_dot1;
	C_matrix_(0,0) = 0;

	g_matrix_(0,0) = (m1*lc1 + m2*l1)*g*std::sin(q1) + m2*lc2*g*std::sin(q1+q2);
	g_matrix_(1,0) = m2*lc2*g*std::sin(q1+q2);

	q_[0] = q1;
	q_[1] = q2;
	q_dot_[0] = q_dot1;
	q_dot_[1] = q_dot2;

//	std::cout<<"joint state: "<< q[0] << "," << q[1] << std::endl;
}

Eigen::Matrix<float,2,2> RobotState::GetMMatrix(void)
{
	return M_matrix_;
}

Eigen::Matrix<float,2,2> RobotState::GetCMatrix(void)
{
	return C_matrix_;
}

Eigen::Matrix<float,2,1> RobotState::GetGMatrix(void)
{
	return g_matrix_;
}
