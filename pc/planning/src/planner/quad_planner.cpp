/*
 * quad_planner.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: rdu
 */

#include <iostream>
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
	if(config.GetMapType().data_model == MapDataModel::QUAD_TREE)
	{
		bool result = qtree_planner_.UpdateMapConfig(config);

		if(result)
		{
			std::cout << "quad tree planner activated" << std::endl;
			active_graph_planner_ = GraphPlannerType::QUADTREE_PLANNER;
		}
	}
	else if(config.GetMapType().data_model == MapDataModel::SQUARE_GRID)
	{
		bool result = sgrid_planner_.UpdateMapConfig(config);

		if(result)
		{
			std::cout << "square grid planner activated" << std::endl;
			active_graph_planner_ = GraphPlannerType::SQUAREGRID_PLANNER;
		}
	}
}

void QuadPlanner::SetStartMapPosition(Position2D pos)
{
	start_pos_.x = pos.x;
	start_pos_.y = pos.y;
}

void QuadPlanner::SetGoalMapPosition(Position2D pos)
{
	goal_pos_.x = pos.x;
	goal_pos_.y = pos.y;
}

std::vector<uint64_t> QuadPlanner::SearchForPath()
{
	std::vector<uint64_t> traj;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		auto traj_vtx = qtree_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			traj.push_back(wp->vertex_id_);
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		auto traj_vtx = sgrid_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			traj.push_back(wp->vertex_id_);
	}

	return traj;
}
