/* 
 * mission_tracker.hpp
 * 
 * Created on: Nov 3, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef MISSION_TRACKER_HPP
#define MISSION_TRACKER_HPP

#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <limits>
#include <cmath>
#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.hpp"
#include "map/geo_mark.h"

namespace librav {

class MissionTracker {
public:
	MissionTracker(std::shared_ptr<lcm::LCM> lcm);
	~MissionTracker() = default;

public:
	int64_t path_id_;
	bool mission_started_;
	bool replan_needed_;
	std::vector<GeoMark> active_path_;

	double remaining_path_length_;
	Position3Dd current_position_;

	void UpdateActivePathWaypoints(std::vector<GeoMark>& path);
	void UpdateCurrentPosition(Position3Dd pos);

private:
	std::shared_ptr<lcm::LCM> lcm_;

	int64_t trajectory_id_;

	double CalcRemainingPathLenght(uint32_t current_idx);
	void LcmMissionInfoHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const librav_lcm_msgs::MissionInfo_t* msg);
};

}

#endif /* MISSION_TRACKER_HPP */
