/*
 * waypoint_path_handler.h
 *
 *  Created on: Nov 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_WAYPOINT_PATH_HANDLER_H_
#define CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_WAYPOINT_PATH_HANDLER_H_

#include <vector>
#include <memory>
#include <atomic>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.h"
#include "common/librav_types.h"

namespace librav {

typedef struct {
	Position3Dd start_pos;
	double start_yaw;
	Position3Dd end_pos;
	double end_yaw;

	double t_start;
	double t_end;
} QuadWaypointTrajSeg;

typedef struct {
	std::vector<QuadWaypointTrajSeg> traj_segs_;

	void clear() { traj_segs_.clear(); };
} QuadWaypointTraj;

class WaypointPathHandler {
public:
	WaypointPathHandler(std::shared_ptr<lcm::LCM> lcm);
	WaypointPathHandler(std::shared_ptr<lcm::LCM> lcm, std::string path_topic);
	~WaypointPathHandler();

	friend class MotionServer;

private:
	std::shared_ptr<lcm::LCM> lcm_;
	std::string path_topic_;

	std::atomic<bool> traj_available_;
	std::atomic<time_stamp> current_sys_time_;
	time_stamp traj_start_time_;

	QuadWaypointTraj wp_traj_;

	void UpdateSystemTime(double t) { current_sys_time_ = t; };

public:
	void LcmPolyTrajMsgHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::Path_t* msg);
	UAVTrajectoryPoint GetDesiredTrajectoryPoint(time_t tstamp);
};

}

#endif /* CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_WAYPOINT_PATH_HANDLER_H_ */
