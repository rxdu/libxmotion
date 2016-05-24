/*
 * motion_server.h
 *
 *  Created on: May 23, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_MOTION_SERVER_H_
#define CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_MOTION_SERVER_H_

#include <common/control_types.h>
#include <vector>


namespace srcl_ctrl {

class MotionServer {
public:
	MotionServer();
	~MotionServer();

private:
	bool goal_completed_;
	UAVTrajectory active_goal_;
	uint64_t waypoint_idx_;
	uint64_t ms_count_;

private:
	UAVTrajectory GenerateTestTrajectory();

public:
	void SetMotionGoal(UAVTrajectory& goal);
	void AbortActiveMotion();
	double ReportActiveMotionProgress();
	void SetGoalCompleted();

	UAVTrajectoryPoint GetCurrentDesiredPose();
};

}


#endif /* CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_MOTION_SERVER_H_ */
