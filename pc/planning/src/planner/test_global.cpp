/*
 * test_local.cpp
 *
 *  Created on: Jul 27, 2016
 *      Author: rdu
 */

#include <iostream>

#include "planner/graph_planner.h"

using namespace srcl_ctrl;

int main(int argc, char** argv)
{
	MapConfig config;

	config.SetMapPath("/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/pc/planning/data/example.png");
//	config.SetMapType(MapDataModel::QUADTREE, 6);
	config.SetMapType(MapDataModel::QUADTREE, 32);

//	GraphPlanner<QuadTree> qt_planner;
//	bool update_result = qt_planner.UpdateMapConfig(config);

	GraphPlanner<SquareGrid> sg_planner;
	bool update_result = sg_planner.UpdateMapConfig(config);

	if(update_result)
	{
		std::cout << "map updated" << std::endl;

		//auto path = qt_planner.Search(5, 94);
		auto path = sg_planner.Search(160, 296);

		for(auto& waypoint:path)
			std::cout << waypoint->vertex_id_ << std::endl;
	}
}



