/*
 * trajectory_generator.h
 *
 *  Created on: Oct 31, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_MISSION_TRAJECTORY_GENERATOR_H_
#define PLANNING_SRC_MISSION_TRAJECTORY_GENERATOR_H_

#include <vector>
#include <memory>
#include <cstdint>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

#include "common/planning_types.h"
#include "quad_flat/quad_polyopt.h"
#include "mission/mission_utils.h"

namespace srcl_ctrl {

class TrajectoryGenerator {
public:
	TrajectoryGenerator(std::shared_ptr<lcm::LCM> lcm);
	~TrajectoryGenerator();

private:
	std::shared_ptr<lcm::LCM> lcm_;
	uint64_t user_path_id_;
	double CalcFlightTime(Position3Dd start, Position3Dd goal, double vel);

private:
	void LcmWaypointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::Path_t* msg);
	void LcmKeyframeSetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::KeyframeSet_t* msg);

public:
	void GenerateTrajectory(KeyframeSet& kfs, uint64_t traj_id);
};

}

#endif /* PLANNING_SRC_MISSION_TRAJECTORY_GENERATOR_H_ */
