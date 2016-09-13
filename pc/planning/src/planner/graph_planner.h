/*
 * graph_planner.h
 *
 *  Created on: Aug 1, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PLANNER_GRAPH_PLANNER_H_
#define PLANNING_SRC_PLANNER_GRAPH_PLANNER_H_

#include <string>
#include <cstdint>
#include <memory>
#include <type_traits>

// opencv
#include "opencv2/opencv.hpp"

// User headers
#include "graph/graph.h"
#include "map/map_type.h"
#include "map/map_config.h"
#include "map/map_utils.h"
#include "geometry/square_grid/square_grid.h"
#include "geometry/quadtree/quad_tree.h"
#include "geometry/sgrid_builder.h"
#include "geometry/qtree_builder.h"
#include "geometry/graph_builder.h"

namespace srcl_ctrl {

enum class GraphPlannerType {
	SQUAREGRID_PLANNER,
	QUADTREE_PLANNER,
	NOT_SPECIFIED
};

template<typename MapDataModel>
class GraphPlanner {
public:
	GraphPlanner():is_ready_(false),
		prev_start_id_(0), prev_goal_id_(0){};
	~GraphPlanner(){};

	typedef typename MapDataModel::node_type MapDataModelNode;

private:
	bool is_ready_;
	uint64_t prev_start_id_;
	uint64_t prev_goal_id_;
	Trajectory_t<MapDataModelNode*> prev_result_;
	Trajectory_t<MapDataModelNode*> empty_path_;

public:
	Map_t<MapDataModel> map_;
	std::shared_ptr<Graph_t<MapDataModelNode*>> graph_;

public:
	template<class T = MapDataModel, typename std::enable_if<std::is_same<T, QuadTree>::value>::type* = nullptr>
	bool UpdateMapConfig(MapConfig config)
	{
		cv::Mat input_image;

		if(MapUtils::ReadImageFromFile(config.GetMapPath(), input_image)) {
			map_ = QTreeBuilder::BuildQuadTreeMap(input_image, config.GetMapType().data_param);
			graph_ = GraphBuilder::BuildFromQuadTree(map_.data_model);

			map_.info.origin_offset_x = config.GetOriginOffsetX();
			map_.info.origin_offset_y = config.GetOriginOffsetY();

			is_ready_ = true;
		}
		else
			is_ready_ = false;

		return is_ready_;
	}

	template<class T = MapDataModel, typename std::enable_if<std::is_same<T, SquareGrid>::value>::type* = nullptr>
	bool UpdateMapConfig(MapConfig config)
	{
		cv::Mat input_image;

		if(MapUtils::ReadImageFromFile(config.GetMapPath(), input_image)) {
			map_ = SGridBuilder::BuildSquareGridMap(input_image, config.GetMapType().data_param);
			graph_ = GraphBuilder::BuildFromSquareGrid(map_.data_model,true);

			map_.info.origin_offset_x = config.GetOriginOffsetX();
			map_.info.origin_offset_y = config.GetOriginOffsetY();

			is_ready_ = true;
		}
		else
			is_ready_ = false;

		return is_ready_;
	}

	template<class T = MapDataModel, typename std::enable_if<std::is_same<T, SquareGrid>::value>::type* = nullptr>
	bool UpdateMapConfig(std::shared_ptr<T> grid)
	{
		cv::Mat input_image;

		graph_ = GraphBuilder::BuildFromSquareGrid(grid,true);
		is_ready_ = true;

		return is_ready_;
	}

	Trajectory_t<MapDataModelNode*> Search(Vertex_t<MapDataModel*>* start, Vertex_t<MapDataModel*>* goal)
	{
		if(is_ready_)
		{
			return graph_->AStarSearch(start, goal);
		}

		return empty_path_;
	}

	Trajectory_t<MapDataModelNode*> Search(uint64_t start, uint64_t goal)
	{
		if(is_ready_)
		{
			return graph_->AStarSearch(start, goal);
		}

		return empty_path_;
	}

	Trajectory_t<MapDataModelNode*> Search(Position2D start, Position2D goal)
	{
		if(is_ready_)
		{
			uint64_t start_id = map_.data_model->GetIDFromPosition(start.x, start.y);
			uint64_t goal_id = map_.data_model->GetIDFromPosition(goal.x, goal.y);

			if(start_id == prev_start_id_ && goal_id == prev_goal_id_)
				return prev_result_;

			auto start_vtx = graph_->GetVertexFromID(start_id);
			auto goal_vtx = graph_->GetVertexFromID(goal_id);

			if(start_vtx == nullptr || goal_vtx == nullptr)
				return prev_result_;

			prev_result_.clear();
			prev_result_ = graph_->AStarSearch(start_vtx, goal_vtx);
			prev_start_id_ = start_id;
			prev_goal_id_ = goal_id;

			if(start_vtx != nullptr && goal_vtx != nullptr)
				return prev_result_;
			else
				return empty_path_;
		}

		return empty_path_;
	}
};

}

#endif /* PLANNING_SRC_PLANNER_GRAPH_PLANNER_H_ */
