/*
 * quadsim_planner.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: rdu
 * Description: 20 by 25 map with sensor range 8, compare heading
 * 
 */

#include <string>
#include <memory>
#include <thread>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.h"
#include "utility/logging/logger.h"
#include "utility/stopwatch/stopwatch.h"
#include "map/map_utils.h"
#include "map/map_config.h"
#include "map/map_info.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "path_repair/sim/virtual_quadrotor.h"

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
	vquad.Load_20by25_Config();
	vquad.SetSensorRange(8);

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

		vquad.CmpStep();
		lcm->handleTimeout(0);
	
		int64_t duration = LOOP_PERIOD - static_cast<int64_t>(timer.mtoc());
		
		if(duration > 0)
			std::this_thread::sleep_for(std::chrono::milliseconds(duration));
	}
}
