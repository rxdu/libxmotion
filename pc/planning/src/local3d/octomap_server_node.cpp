/*
 * octomap_server_node.cpp
 *
 *  Created on: May 27, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <ctime>
#include <chrono>
#include <thread>

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

	bool tree_saved = false;
	clock_t start_time;
	start_time = clock();

	while(true)
	{
		lcm->handleTimeout(0);

//		double duration =  double(clock() - start_time)/CLOCKS_PER_SEC;
//		if(duration > 10 && !tree_saved) {
//			server.SaveTreeToFile("octree_set2.bt");
//			tree_saved = true;
//		}

//		 delay for some time
//		std::chrono::seconds timespan(10); // or whatever
//		std::this_thread::sleep_for(timespan);
	}

	return 0;
}
