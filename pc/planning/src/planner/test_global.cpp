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
//	MapConfig config;
//
//	config.SetMapPath("/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/pc/planning/data/example.png");
////	config.SetMapType(MapDataModel::QUADTREE, 6);
//	config.SetMapType(MapDataModel::QUADTREE, 32);
//
////	GraphPlanner<QuadTree> qt_planner;
////	bool update_result = qt_planner.UpdateMapConfig(config);
//
//	GraphPlanner<SquareGrid> sg_planner;
//	bool update_result = sg_planner.UpdateMapConfig(config);
//
//	if(update_result)
//	{
//		std::cout << "map updated" << std::endl;
//
//		//auto path = qt_planner.Search(5, 94);
//		auto path = sg_planner.Search(160, 296);
//
//		for(auto& waypoint:path)
//			std::cout << waypoint->vertex_id_ << std::endl;
//	}

	std::shared_ptr<SquareGrid> grid = MapUtils::CreateSquareGrid(12,12,95);

	// set occupancy for cells
	for(int i = 52; i <= 57; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 88; i <= 93; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 74; i <= 75; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	for(int i = 0; i < 8; i++)
		grid->SetCellOccupancy(i,10, OccupancyType::OCCUPIED);

	for(int i = 24; i <= 28; i++)
		grid->SetCellOccupancy(i, OccupancyType::OCCUPIED);

	grid->SetCellOccupancy(58, OccupancyType::OCCUPIED);
	grid->SetCellOccupancy(87, OccupancyType::OCCUPIED);
	grid->SetCellOccupancy(22, OccupancyType::OCCUPIED);
	grid->SetCellOccupancy(34, OccupancyType::OCCUPIED);
	grid->SetCellOccupancy(46, OccupancyType::OCCUPIED);
	grid->SetCellOccupancy(118, OccupancyType::OCCUPIED);
	grid->SetCellOccupancy(119, OccupancyType::OCCUPIED);

	grid->SetCellOccupancy(7, OccupancyType::OCCUPIED);
	grid->SetCellOccupancy(19, OccupancyType::OCCUPIED);
	grid->SetCellOccupancy(31, OccupancyType::OCCUPIED);

	grid->SetCellOccupancy(66, OccupancyType::OCCUPIED);
	grid->SetCellOccupancy(81, OccupancyType::OCCUPIED);

	GraphPlanner<SquareGrid> sg_planner;
	bool update_result = sg_planner.UpdateMapConfig(grid);

	if(update_result)
	{
		std::cout << "map updated" << std::endl;

		//auto path = qt_planner.Search(5, 94);
		auto path = sg_planner.Search(0, 143);

		for(auto& waypoint:path)
			std::cout << waypoint->vertex_id_ << std::endl;
	}
}



