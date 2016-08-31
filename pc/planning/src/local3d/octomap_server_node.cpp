/*
 * octomap_server_node.cpp
 *
 *  Created on: May 27, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "local3d/octomap_server.h"

using namespace srcl_ctrl;

int main(int argc, char** argv)
{
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return 1;
	}

	OctomapServer server(lcm);

	std::cout << "INFO: Octomap server started." << std::endl;

//	uint64_t i = 0;

	while(true)
	{
//		if(i == 5000000) {
//			server.SaveTreeToFile();
//			return 0;
//		}
//
//		i++;

		lcm->handleTimeout(0);
	}

	return 0;
}
