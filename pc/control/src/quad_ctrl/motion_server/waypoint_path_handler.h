/*
 * waypoint_path_handler.h
 *
 *  Created on: Nov 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_WAYPOINT_PATH_HANDLER_H_
#define CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_WAYPOINT_PATH_HANDLER_H_


// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

namespace srcl_ctrl {

class WaypointPathHandler {
public:
	WaypointPathHandler(std::shared_ptr<lcm::LCM> lcm);
	WaypointPathHandler(std::shared_ptr<lcm::LCM> lcm, std::string path_topic);
	~WaypointPathHandler();

	friend class MotionServer;

private:
	std::shared_ptr<lcm::LCM> lcm_;
	std::string path_topic_;
	double step_size_;

public:
	void LcmPolyTrajMsgHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::Path_t* msg);
	UAVTrajectoryPoint GetDesiredTrajectoryPoint(time_t tstamp);
};

}

#endif /* CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_WAYPOINT_PATH_HANDLER_H_ */
