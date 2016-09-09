/*
 * test_local.cpp
 *
 *  Created on: Jul 27, 2016
 *      Author: rdu
 */

//#include <rrts_planner.h>
#include "planner/rrts_planner_local.h"

using namespace srcl_ctrl;

int main(int argc, char** argv)
{
	RRTStarPlannerLocal planner;

	//planner.ConfigLocalPlanner();
	//planner.SearchSolution();
	std::vector<Position3Dd> path3d;
	Position3Dd start;
	Position3Dd goal;

	start.x = 0.02;
	start.y = 0.01;
	start.z = 0.01;

	goal.x = 2.5;
	goal.y = 0.5;
	goal.z = 0.3;

	planner.SearchSolution( start, goal, 10, path3d);
}



