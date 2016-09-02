/*
 * robot_state_base.h
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_COMMON_ROBOT_STATE_BASE_H_
#define CONTROL_SRC_COMMON_ROBOT_STATE_BASE_H_

#include "common/control_types.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace srcl_ctrl{

class RobotStateBase {
public:
	RobotStateBase();
	virtual ~RobotStateBase();

public:
	Point3f position_;
	Eigen::Quaterniond quat_;

	void UpdateRobotState(const DataFromQuad &new_data);
};

#endif /* CONTROL_SRC_COMMON_ROBOT_STATE_BASE_H_ */
