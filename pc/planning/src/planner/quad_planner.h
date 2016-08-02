/*
 * quad_planner.h
 *
 *  Created on: Aug 1, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PLANNER_QUAD_PLANNER_H_
#define PLANNING_SRC_PLANNER_QUAD_PLANNER_H_

#include "common/planning_types.h"
#include "planner/graph_planner.h"
#include "planner/rrts_planner.h"

namespace srcl_ctrl {

enum class GraphPlannerType {
	SQUAREGRID_PLANNER,
	QUADTREE_PLANNER,
	NOT_SPECIFIED
};

class QuadPlanner{
public:
	QuadPlanner();
	~QuadPlanner();

private:
	// planners
	GraphPlanner<QuadTree> qtree_planner_;
	GraphPlanner<SquareGrid> sgrid_planner_;

	RRTStarPlanner local_planner_;

	// planning parameters
	Position2D start_pos_;
	Position2D goal_pos_;

public:
	GraphPlannerType active_graph_planner_;

public:
	void ConfigGraphPlanner(MapConfig config);
	void SetStartMapPosition(Position2D pos);
	void SetGoalMapPosition(Position2D pos);

	std::vector<uint64_t> SearchForPath();
};

}


#endif /* PLANNING_SRC_PLANNER_QUAD_PLANNER_H_ */
