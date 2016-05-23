/*
 * motion_server.h
 *
 *  Created on: May 23, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_MOTION_SERVER_H_
#define CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_MOTION_SERVER_H_

#include <vector>

#include "common/robot_datatypes.h"

namespace srcl_ctrl {

class MotionServer {
public:
	MotionServer();
	~MotionServer();

private:
	std::vector<TrajectoryPoint> active_goal_;

public:
	void SetMotionGoal(std::vector<TrajectoryPoint>& goal);
	void AbortActiveMotion();
	double GetActiveMotionProgress();
};

}


#endif /* CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_MOTION_SERVER_H_ */
