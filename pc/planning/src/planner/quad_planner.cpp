/*
 * quad_planner.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: rdu
 */

#include "planner/quad_planner.h"

using namespace srcl_ctrl;

QuadPlanner::QuadPlanner():
		active_graph_planner_(GraphPlannerType::NOT_SPECIFIED)
{

}

QuadPlanner::~QuadPlanner()
{

}

void QuadPlanner::ConfigGraphPlanner(MapConfig config)
{
	if(config.GetMapType().data_model == MapDataModel::QUADTREE)
	{
		bool result = qtree_planner_.UpdateMapConfig(config);

		if(result)
			active_graph_planner_ = GraphPlannerType::QUADTREE_PLANNER;
	}
	else if(config.GetMapType().data_model == MapDataModel::SQUARE_GRID)
	{
		bool result = sgrid_planner_.UpdateMapConfig(config);

		if(result)
			active_graph_planner_ = GraphPlannerType::SQUARE_GRID_PLANNER;
	}
}
