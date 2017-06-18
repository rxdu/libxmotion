/*
 * waypoint_manager.cpp
 *
 *  Created on: Oct 31, 2016
 *      Author: rdu
 */

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include <path_manager.h>
#include "lcmtypes/srcl_ctrl.hpp"


using namespace srcl_ctrl;

int main(int argc, char* argv[])
{
	// send data for visualization
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	std::shared_ptr<PathManager> wp_m = std::make_shared<PathManager>(lcm);

	while(true)
	{
		lcm->handleTimeout(0);
	}
}


