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

void TestCase1_Config(PathRepair& qplanner)
{
	// config 2d map
	// std::string image_dir = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/map_path_repair.png";
	// MapConfig map_config;
	// map_config.SetMapPath(image_dir);
	// map_config.SetMapType(MapDataModel::SQUARE_GRID, 32);
	// map_config.SetOriginOffset(2.5, 2.5);
	// qplanner.ConfigGraphPlanner(map_config, 5.0, 5.0);

	// position update
	qplanner.EnablePositionAutoUpdate(true);

	// set start and goal
	qplanner.SetGoalRefWorldPosition(Position2Dd(1.8, -2.0));
	qplanner.SetGoalHeightRange(0.5, 2.5);
}

int main(int argc, char* argv[])
{
	// set up network first
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
	if(!lcm->good())
	{
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	// init quadrotor planner
	PathRepair qplanner(lcm);

	// TestCase1_Config(qplanner);

	//LoggingHelper& logging_helper = LoggingHelper::GetInstance("quadsim_hummingbird", "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/log");

	while(true)
	{
		// if(qplanner.update_global_plan_)
		// 	qplanner.UpdateGlobalPath();

		lcm->handleTimeout(0);
	}
}
