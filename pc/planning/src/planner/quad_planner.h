/*
 * quad_planner.h
 *
 *  Created on: Aug 1, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PLANNER_QUAD_PLANNER_H_
#define PLANNING_SRC_PLANNER_QUAD_PLANNER_H_

#include <graph_planner.h>
#include <rrts_planner.h>

namespace srcl_ctrl {

enum class GraphPlannerType {
	SQUARE_GRID_PLANNER,
	QUADTREE_PLANNER,
	NOT_SPECIFIED
};

class QuadPlanner{
public:
	QuadPlanner();
	~QuadPlanner();

private:
	GraphPlanner<QuadTree> qtree_planner_;
	GraphPlanner<SquareGrid> sgrid_planner_;

	GraphPlannerType active_graph_planner_;

	RRTStarPlanner local_planner_;

public:
	void ConfigGraphPlanner(MapConfig config);
	std::vector<Position2D> SearchForGlobalPath();
};

}


#endif /* PLANNING_SRC_PLANNER_QUAD_PLANNER_H_ */
