/*
 * graph_planner_v2.h
 *
 *  Created on: Sep 5, 2017
 *      Author: rdu
 */

#ifndef QUADROTOR_PATH_REPAIR_SG_GRAPH_PLANNER_H_
#define QUADROTOR_PATH_REPAIR_SG_GRAPH_PLANNER_H_

#include <string>
#include <cstdint>
#include <memory>
#include <type_traits>

// opencv
#include "opencv2/opencv.hpp"

// User headers
#include "planning/graph/graph.h"
#include "planning/graph/astar.h"
#include "planning/map/map_type.h"
#include "planning/map/map_config.h"
#include "planning/map/map_utils.h"

#include "geometry/square_grid.h"
#include "geometry/quad_tree.h"
#include "geometry/sgrid_builder.h"
#include "geometry/qtree_builder.h"
#include "geometry/graph_builder.h"

namespace librav {

class SGGraphPlanner {
public:
	SGGraphPlanner():is_ready_(false),
		prev_start_id_(0), prev_goal_id_(0){};

	Map_t<SquareGrid> map_;
	std::shared_ptr<Graph_t<SquareCell*>> graph_;

public:
	bool UpdateMapConfig(MapConfig config);
	bool UpdateMapConfig(std::shared_ptr<SquareGrid> grid);
	Path_t<SquareCell*> Search(Vertex_t<SquareCell*>* start, Vertex_t<SquareCell*>* goal);
	Path_t<SquareCell*> Search(uint64_t start, uint64_t goal);
	Path_t<SquareCell*> Search(Position2Di start, Position2Di goal);

private:
	bool is_ready_;
	uint64_t prev_start_id_;
	uint64_t prev_goal_id_;
	Path_t<SquareCell*> prev_result_;
	Path_t<SquareCell*> empty_path_;
};

}

#endif /* QUADROTOR_PATH_REPAIR_SG_GRAPH_PLANNER_H_ */
