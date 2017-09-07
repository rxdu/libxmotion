/*
 * quadsim_planner.cpp
 *
 *  Created on: Sep 7, 2016
 *      Author: rdu
 */

#include <string>
#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/planning_types.h"
#include "utility/logging/logger.h"
#include "planning/map/map_utils.h"
#include "planning/map/map_config.h"
#include "planning/map/map_info.h"
#include "planning/geometry/graph_builder.h"
#include "planning/geometry/sgrid_builder.h"
#include "quadrotor/path_repair/path_repair.h"

using namespace librav;

void Test_30by50_Config(PathRepair &qplanner)
{
	qplanner.EnablePositionAutoUpdate(true);
	qplanner.SetStartPosition(Position2D(0, 0));
	qplanner.SetGoalPosition(Position2D(29, 49));
	qplanner.SetGoalHeightRange(0.5, 2.5);
}

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
	PathRepair qplanner(lcm);

	Test_30by50_Config(qplanner);

	//LoggingHelper& logging_helper = LoggingHelper::GetInstance("quadsim_hummingbird", "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/log");

	while (true)
	{
		if (qplanner.update_global_plan_)
		{
			auto path = qplanner.UpdateGlobalPathID();
			if (!path.empty())
				std::cout << "Path found" << std::endl;
			else
				std::cout << "Empty path" << std::endl;
		}

		lcm->handleTimeout(0);
	}
}
