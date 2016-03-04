/*
 * robot_state.h
 *
 *  Created on: Mar 1, 2016
 *      Author: rdu
 */

#ifndef NAVIGATION_ROBOT_STATE_H_
#define NAVIGATION_ROBOT_STATE_H_

#include "library/eigen3/Eigen/Core"
#include "library/eigen3/Eigen/Geometry"

#include "vrep_client/robot_datatypes.h"

namespace srcl_ctrl{

class RobotState {
public:
	RobotState();
	~RobotState();

public:
	Point3 position;
	Point3 velocity;
	Point3 orientation;
	Eigen::Quaterniond quat;
	Point3 rotation_rate;

private:
	Eigen::Matrix<float,2,2> M_matrix_;

public:
	double w_h;
	const float g;

	// quadrotor parameters
	double mass;
	double arm_length;
	double kF;
	double kM;

	double sim_step;

public:
	void UpdateRobotState(const DataFromRobot &new_data);
	Eigen::Matrix<float,2,2> GetMMatrix(void);
};

}

#endif /* NAVIGATION_ROBOT_STATE_H_ */
