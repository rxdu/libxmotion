/*
 * robot_state.h
 *
 *  Created on: Jul 29, 2015
 *      Author: rdu
 */

#ifndef ROBOT_STATE_ROBOT_STATE_H_
#define ROBOT_STATE_ROBOT_STATE_H_

#include <vrep_client/robot_datatypes.h>
#include "library/eigen3/Eigen/Core"

namespace srcl_ctrl
{
class RobotState
{
public:
	RobotState();
	~RobotState();

private:
	const float l1 = 0.26;
	const float l2 = 0.26;
	const float lc1 = 0.0983;
	const float lc2 = 0.0229;
	const float m1 = 6.5525;
	const float m2 = 2.0458;
	const float I1 = 0.1213;
	const float I2 = 0.0116;
	const float g = 9.81;

public:
	float q_[2];
	float q_dot_[2];

private:
	Eigen::Matrix<float,2,2> M_matrix_;
	Eigen::Matrix<float,2,2> C_matrix_;
	Eigen::Matrix<float,2,1> g_matrix_;

public:
	void UpdateRobotState(const JointState &joint);
	Eigen::Matrix<float,2,2> GetMMatrix(void);
	Eigen::Matrix<float,2,2> GetCMatrix(void);
	Eigen::Matrix<float,2,1> GetGMatrix(void);
};

}

#endif /* ROBOT_STATE_ROBOT_STATE_H_ */
