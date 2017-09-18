/*
 * quadsim_planner.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: rdu
 */

#include <string>
#include <memory>
#include <thread>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/planning_types.h"
#include "utility/logging/logger.h"
#include "stopwatch/stopwatch.h"
#include "planning/map/map_utils.h"
#include "planning/map/map_config.h"
#include "planning/map/map_info.h"
#include "planning/geometry/graph_builder.h"
#include "planning/geometry/sgrid_builder.h"
#include "quadrotor/path_repair/sim/virtual_quadrotor.h"

using namespace librav;

#define LOOP_PERIOD 1000	// ms

int main(int argc, char *argv[])
{
	// set up network first
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
	if (!lcm->good())
	{
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	// init quadrotor planner
	VirtualQuadrotor vquad(lcm);
	// vquad.Load_5by5_Config();
	vquad.Load_10by10_Config();
	// vquad.Load_30by50_Config();	

	// should not start simulation if configuration is not complete
	if (!vquad.IsReady())
	{
		std::cerr << "ERROR: Incomplete configuration for the simulation" << std::endl;
		return -1;
	}

	stopwatch::StopWatch timer;

	// simulation loop
	while (true)
	{		
		timer.tic();

		vquad.Step();
		lcm->handleTimeout(0);
	
		int64_t duration = static_cast<int64_t>(timer.mtoc());
		std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD - duration));
	}
}
