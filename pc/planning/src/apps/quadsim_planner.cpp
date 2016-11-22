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

#ifdef ENABLE_G3LOG
#include "ctrl_utils/logging/logging_helper.h"
#endif

#include "common/planning_types.h"
#include "map/map_utils.h"
#include "map/map_config.h"
#include "map/map_info.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "path_repair/quad_path_repair.h"

using namespace srcl_ctrl;

void TestCase1_Config(QuadPathRepair& qplanner)
{
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/experiments/map_path_repair.png";

	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 16);
	//	map_config.SetMapType(MapDataModel::QUAD_TREE, 6);
	map_config.SetOriginOffset(2.5, 2.5);

	qplanner.ConfigGraphPlanner(map_config, 5.0, 5.0);
	qplanner.EnablePositionAutoUpdate(true);

	qplanner.SetGoalRefWorldPosition(Position2Dd(1.8, -2.0));
	qplanner.SetDesiredHeight(0.80);
}

void TestCase2_Config(QuadPathRepair& qplanner)
{
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/experiments/map_testcase2.png";

	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 16);
	//	map_config.SetMapType(MapDataModel::QUAD_TREE, 6);
	map_config.SetOriginOffset(10.0, 12.5);
	//map_config.SetOriginOffset(12.5, 10.0);

	qplanner.ConfigGraphPlanner(map_config, 20.0, 25.0);
	qplanner.EnablePositionAutoUpdate(true);

	qplanner.SetGoalRefWorldPosition(Position2Dd(11.0, -8.5));
	qplanner.SetDesiredHeight(1.8);
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
	QuadPathRepair qplanner(lcm);

	//TestCase1_Config(qplanner);
	TestCase2_Config(qplanner);

#ifdef ENABLE_G3LOG
	LoggingHelper& logging_helper = LoggingHelper::GetInstance("quadsim_hummingbird", "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/log");
#endif

	if(qplanner.active_graph_planner_ == GraphPlannerType::NOT_SPECIFIED)
	{
		std::cout << "failed to init quad planner" << std::endl;
		return -1;
	}

	while(true)
	{
		if(qplanner.update_global_plan_)
			qplanner.UpdateGlobalPath();

		lcm->handleTimeout(0);
	}
}


