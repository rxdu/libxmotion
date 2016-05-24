/*
 * robot_state.h
 *
 *  Created on: Mar 1, 2016
 *      Author: rdu
 */

#ifndef NAVIGATION_ROBOT_STATE_H_
#define NAVIGATION_ROBOT_STATE_H_

#include "common/control_types.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"


namespace srcl_ctrl{

class RobotState {
public:
	RobotState();
	~RobotState();

public:
	Point3f position_;
	Point3f velocity_;
	Point3f orientation_;
	Eigen::Quaterniond quat_;
	Point3f rotation_rate_;

	QuadFlightType quad_flight_type_;

private:
	Eigen::Matrix<float,2,2> M_matrix_;
	Eigen::Quaterniond last_quat_;
	Point3f last_orientation_;

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
	void UpdateRobotState(const DataFromQuad &new_data);
	Eigen::Matrix<float,2,2> GetMMatrix(void);
};

}

#endif /* NAVIGATION_ROBOT_STATE_H_ */
