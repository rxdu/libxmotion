/*
 * motion_server.h
 *
 *  Created on: May 23, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_MOTION_SERVER_H_
#define CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_MOTION_SERVER_H_

#include <vector>
#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

#include "common/control_types.h"
#include "quad_ctrl/motion_server/quad_poly_traj_handler.h"

namespace srcl_ctrl {

class MotionServer {
public:
	MotionServer();
	MotionServer(std::shared_ptr<lcm::LCM> lcm);
	~MotionServer();

private:
	std::shared_ptr<lcm::LCM> lcm_;
	std::unique_ptr<QuadPolyTrajHandler> polytraj_handler_;

private:
	bool goal_completed_;
	UAVTrajectory active_goal_;
	uint64_t ms_count_;
	uint64_t waypoint_idx_;
	time_stamp current_sys_time_;

private:
	UAVTrajectory GenerateTestTrajectory();

public:
	void LcmSysTimeHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::TimeStamp_t* msg);

	void LcmGoalHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::UAVTrajectory_t* msg);
	void SetMotionGoal(UAVTrajectory& goal);
	void AbortActiveMotion();
	double ReportActiveMotionProgress();
	void SetGoalCompleted();

	UAVTrajectoryPoint GetCurrentDesiredPose();
	UAVTrajectoryPoint GetCurrentDesiredState(time_stamp t);
};

}


#endif /* CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_MOTION_SERVER_H_ */
