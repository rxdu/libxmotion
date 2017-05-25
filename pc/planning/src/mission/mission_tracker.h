/*
 * mission_tracker.h
 *
 *  Created on: Nov 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_MISSION_MISSION_TRACKER_H_
#define PLANNING_SRC_MISSION_MISSION_TRACKER_H_

#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <limits>
#include <cmath>
#include <memory>

#include "eigen3/Eigen/Geometry"

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/srcl_ctrl.hpp"

#include "graph/graph.h"
#include "common/planning_types.h"
#include "geometry/geo_mark.h"

namespace srcl_ctrl {

class MissionTracker {
public:
	MissionTracker(std::shared_ptr<lcm::LCM> lcm);
	~MissionTracker(){};

private:
	std::shared_ptr<lcm::LCM> lcm_;

	int64_t trajectory_id_;

	double CalcRemainingPathLenght(uint32_t current_idx);
	void LcmMissionInfoHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::MissionInfo_t* msg);

public:
	int64_t path_id_;
	bool mission_started_;
	bool replan_needed_;
	std::vector<GeoMark> active_path_;

	double remaining_path_length_;
	Position3Dd current_position_;

	void UpdateActivePathWaypoints(std::vector<GeoMark>& path);
	void UpdateCurrentPosition(Position3Dd pos);

};

}

#endif /* PLANNING_SRC_MISSION_MISSION_TRACKER_H_ */
