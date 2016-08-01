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

class QuadPlanner{
public:
	QuadPlanner();
	~QuadPlanner();

private:
	//GlobalPlanner global_planner_;
	RRTStarPlanner local_planner_;

public:
	void ConfigQuadPlanner();
};

}


#endif /* PLANNING_SRC_PLANNER_QUAD_PLANNER_H_ */
