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

#include "quad_sim/quad_sim_data.h"
#include "quad_ctrl/controller/quad_types.h"

namespace srcl_ctrl{

class QuadState {
public:
	QuadState();
	~QuadState();

public:
	Point3f position_;
	Point3f velocity_;
	Point3f orientation_;
	Eigen::Quaterniond quat_;
	Point3f rotation_rate_;
	std::vector<Point3f> laser_points_;

	QuadFlightType quad_flight_type_;

public:
	double w_h_;
	const float g_;

	// quadrotor parameters
	const double mass_;
	const double arm_length_;
	const double kF_;
	const double kM_;

	double sim_step_;

public:
	void UpdateRobotState(const DataFromQuad &new_data);
	void UpdateRobotState(const QuadDataFromSim &new_data);
};

}

#endif /* NAVIGATION_ROBOT_STATE_H_ */
