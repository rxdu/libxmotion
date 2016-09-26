/*
 * test_polyclient.cpp
 *
 *  Created on: Sep 26, 2016
 *      Author: rdu
 */

#include <iostream>
#include <string>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

#include "motion_client/poly_motion_client.h"

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

	PolyMotionClient client(lcm,"quad_planner/polynomial_curve", "quad_controller/quad_motion_service");

	while(true)
	{
		lcm->handleTimeout(0);
	}
}


