/*
 * quadsim_planner.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: rdu
 * Description: 15 by 20 map with sensor range 12
 * 
 */

#include <string>
#include <memory>
#include <thread>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.hpp"
#include "logging/logger.hpp"
#include "stopwatch/stopwatch.h"
#include "path_repair/sim/virtual_quadrotor.h"

using namespace librav;

#define LOOP_PERIOD 500	// ms

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
	vquad.Load_15by20_Config();
	vquad.SetSensorRange(12);

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
	
		int64_t duration = LOOP_PERIOD - static_cast<int64_t>(timer.mtoc());
		
		if(duration > 0)
			std::this_thread::sleep_for(std::chrono::milliseconds(duration));
	}
}
