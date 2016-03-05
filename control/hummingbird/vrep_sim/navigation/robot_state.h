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
	Point3 position_;
	Point3 velocity_;
	Point3 orientation_;
	Eigen::Quaterniond quat_;
	Point3 rotation_rate_;

private:
	Eigen::Matrix<float,2,2> M_matrix_;
	Eigen::Quaterniond last_quat_;
	Point3 last_orientation_;

public:
	double w_h_;
	const float g_;
	const float max_euler_change_;

	// quadrotor parameters
	const double mass_;
	const double arm_length_;
	const double kF_;
	const double kM_;

	double sim_step_;

private:
	unsigned int invert_quat;

public:
	void UpdateRobotState(const DataFromRobot &new_data);
	Eigen::Matrix<float,2,2> GetMMatrix(void);
};

}

#endif /* NAVIGATION_ROBOT_STATE_H_ */
