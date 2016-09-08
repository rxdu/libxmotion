/*
 * quad_planner.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: rdu
 */

#include <iostream>

#include "planner/quad_planner.h"
#include "map/map_utils.h"

using namespace srcl_ctrl;

QuadPlanner::QuadPlanner():
		active_graph_planner_(GraphPlannerType::NOT_SPECIFIED),
		world_size_set_(false),
		auto_update_pos_(true)
{

}

QuadPlanner::QuadPlanner(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		active_graph_planner_(GraphPlannerType::NOT_SPECIFIED),
		world_size_set_(false),
		auto_update_pos_(true)
{
	if(!lcm_->good())
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
	else {
		lcm_->subscribe("vis_data_quad_transform",&QuadPlanner::LcmTransformHandler, this);
	}
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
			this->ConfigRRTSOccupancyMap(this->qtree_planner_.map_.padded_image, this->qtree_planner_.map_.info);
			active_graph_planner_ = GraphPlannerType::QUADTREE_PLANNER;
		}
	}
	else if(config.GetMapType().data_model == MapDataModel::SQUARE_GRID)
	{
		bool result = sgrid_planner_.UpdateMapConfig(config);

		if(result)
		{
			std::cout << "square grid planner activated" << std::endl;
			this->ConfigRRTSOccupancyMap(this->sgrid_planner_.map_.padded_image, this->sgrid_planner_.map_.info);
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

void QuadPlanner::SetStartMapWorldPosition(Position2Dd pos)
{
	Position2D mpos;
	mpos = MapUtils::CoordinatesFromMapWorldToMap(pos, GetActiveMapInfo());
	start_pos_.x = mpos.x;
	start_pos_.y = mpos.y;
}

void QuadPlanner::SetGoalMapWorldPosition(Position2Dd pos)
{
	Position2D mpos;
	mpos = MapUtils::CoordinatesFromMapWorldToMap(pos, GetActiveMapInfo());
	goal_pos_.x = mpos.x;
	goal_pos_.y = mpos.y;
}

void QuadPlanner::SetStartRefWorldPosition(Position2Dd pos)
{
	Position2Dd mpos;
	mpos = MapUtils::CoordinatesFromRefWorldToMapWorld(pos, GetActiveMapInfo());

	SetStartMapWorldPosition(mpos);
}

void QuadPlanner::SetGoalRefWorldPosition(Position2Dd pos)
{
	Position2Dd mpos;
	mpos = MapUtils::CoordinatesFromRefWorldToMapWorld(pos, GetActiveMapInfo());

	SetGoalMapWorldPosition(mpos);
}

std::vector<uint64_t> QuadPlanner::SearchForGlobalPath()
{
	std::vector<uint64_t> traj;

//	std::cout << "----> start: " << start_pos_.x << " , " << start_pos_.y << std::endl;
//	std::cout << "----> goal: " << goal_pos_.x << " , " << goal_pos_.y << std::endl;

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

bool QuadPlanner::SearchForLocalPath(Position2Dd start, Position2Dd goal, double time_limit, std::vector<Position2Dd>& path2d)
{
	if(!world_size_set_)
	{
		std::cerr << "the size of the world needs to be specified" << std::endl;
		return false;
	}

	return local_planner_.SearchSolution(start, goal, time_limit, path2d);
}

void QuadPlanner::ConfigRRTSOccupancyMap(cv::Mat map, MapInfo info)
{
	local_planner_.UpdateOccupancyMap(map, info);
}

void QuadPlanner::SetRealWorldSize(double x, double y)
{
	qtree_planner_.map_.info.SetWorldSize(x, y);
	sgrid_planner_.map_.info.SetWorldSize(x, y);

	// update the map info inside the rrts planner
	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		this->ConfigRRTSOccupancyMap(this->qtree_planner_.map_.padded_image, this->qtree_planner_.map_.info);
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		this->ConfigRRTSOccupancyMap(this->sgrid_planner_.map_.padded_image, this->sgrid_planner_.map_.info);
	}

	world_size_set_ = true;

	srcl_msgs::Graph_t graph_msg = GenerateLcmGraphMsg();
	lcm_->publish("quadsim/quad_planner_graph", &graph_msg);
}

cv::Mat QuadPlanner::GetActiveMap()
{
	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		return qtree_planner_.map_.padded_image;
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		return sgrid_planner_.map_.padded_image;
	}
	else
		return cv::Mat::zeros(10, 10, CV_8UC1);
}

MapInfo QuadPlanner::GetActiveMapInfo()
{
	MapInfo empty_info;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		return qtree_planner_.map_.info;
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		return sgrid_planner_.map_.info;
	}
	else
		return empty_info;
}

std::shared_ptr<Graph_t<RRTNode>> QuadPlanner::GetLocalPlannerVisGraph()
{
	return local_planner_.rrts_vis_graph_;
}

void QuadPlanner::LcmTransformHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_msgs::QuadrotorTransform* msg)
{
	Position2Dd rpos;
	rpos.x = msg->base_to_world.position[0];
	rpos.y = msg->base_to_world.position[1];

//	Position2Dd mpos;
//	mpos = MapUtils::CoordinatesFromRefWorldToMapWorld(rpos, GetActiveMapInfo());
//
//	std::cout << "quadrotor position in sim: " << msg->base_to_world.position[0] << " , "
//			<< msg->base_to_world.position[1] << " , "
//			<< msg->base_to_world.position[2] << std::endl;
//	std::cout << "quadrotor position in image world: " << mpos.x << " , "
//				<< mpos.y << std::endl;

	if(auto_update_pos_)
	{
		SetStartRefWorldPosition(rpos);
	}
}

srcl_msgs::Graph_t QuadPlanner::GenerateLcmGraphMsg()
{
	srcl_msgs::Graph_t graph_msg;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		graph_msg.vertex_num = this->qtree_planner_.graph_->GetGraphVertices().size();
		for(auto& vtx : this->qtree_planner_.graph_->GetGraphVertices())
		{
			srcl_msgs::Vertex_t vertex;
			vertex.id = vtx->vertex_id_;

			Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapToRefWorld(vtx->bundled_data_->location_, qtree_planner_.map_.info);
			vertex.position[0] = ref_world_pos.x;
			vertex.position[1] = ref_world_pos.y;

			graph_msg.vertices.push_back(vertex);
		}

		graph_msg.edge_num = this->qtree_planner_.graph_->GetGraphUndirectedEdges().size();
		for(auto& eg : this->qtree_planner_.graph_->GetGraphUndirectedEdges())
		{
			srcl_msgs::Edge_t edge;
			edge.id_start = eg.src_->vertex_id_;
			edge.id_end = eg.dst_->vertex_id_;

			graph_msg.edges.push_back(edge);
		}
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		graph_msg.vertex_num = this->sgrid_planner_.graph_->GetGraphVertices().size();
		for(auto& vtx : this->sgrid_planner_.graph_->GetGraphVertices())
		{
			srcl_msgs::Vertex_t vertex;
			vertex.id = vtx->vertex_id_;

			Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapToRefWorld(vtx->bundled_data_->location_, sgrid_planner_.map_.info);
			vertex.position[0] = ref_world_pos.x;
			vertex.position[1] = ref_world_pos.y;

			graph_msg.vertices.push_back(vertex);
		}

		graph_msg.edge_num = this->sgrid_planner_.graph_->GetGraphUndirectedEdges().size();
		for(auto& eg : this->sgrid_planner_.graph_->GetGraphUndirectedEdges())
		{
			srcl_msgs::Edge_t edge;
			edge.id_start = eg.src_->vertex_id_;
			edge.id_end = eg.dst_->vertex_id_;

			graph_msg.edges.push_back(edge);
		}
	}

	return graph_msg;
}
