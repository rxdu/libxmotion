/*
 * waypoint_manager.h
 *
 *  Created on: Oct 31, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PATH_REPAIR_WAYPOINT_MANAGER_H_
#define PLANNING_SRC_PATH_REPAIR_WAYPOINT_MANAGER_H_

#include <vector>
#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

#include "common/planning_types.h"
#include "quad_flat/quad_polyopt.h"

namespace srcl_ctrl {

class WaypointManager {
public:
	WaypointManager(std::shared_ptr<lcm::LCM> lcm);
	~WaypointManager();

private:
	std::shared_ptr<lcm::LCM> lcm_;
	QuadPolyOpt traj_opt_;
	std::vector<Position3Dd> last_path_;

	std::vector<Position3Dd> WaypointSelector(std::vector<Position3Dd>& wps);

private:
	void LcmWaypointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::Path_t* msg);

public:
	void CalcTrajFromWaypoints(std::vector<Position3Dd>& wps);
};

}

#endif /* PLANNING_SRC_PATH_REPAIR_WAYPOINT_MANAGER_H_ */
