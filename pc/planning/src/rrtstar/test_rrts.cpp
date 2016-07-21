/*
 * test_rrts.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#include <iostream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/ScopedState.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "rrtstar/rrtstar_kd.h"

using namespace ompl;
using namespace srcl_ctrl;

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(int argc, char** argv)
{
	// define the R3 space for (x,y,z)
	ompl::base::StateSpacePtr r3(new ompl::base::RealVectorStateSpace(3));
	ompl::base::RealVectorBounds bounds(3);
	bounds.setLow(-100);
	bounds.setHigh(100);
	r3->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	// define the SO(2) space for (yaw)
	ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());

	// add two spaces to get a R3*SO(2) space
	ompl::base::StateSpacePtr flat_space = r3 + so2;

	// create an instance of planner
	ob::SpaceInformationPtr si(new ob::SpaceInformation(flat_space));
//	base::PlannerPtr planner(new RRTStarKD(si));
	base::PlannerPtr planner(new og::RRTstar(si));

	// define the problem to be solved
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	ompl::base::ScopedState<> start(flat_space);
	start.random();
	ob::ScopedState<> goal(flat_space);
	goal.random();

	std::cout << "\n Start: " << std::endl;
	std::cout << start;

	std::cout << "\n Goal: " << std::endl;
	std::cout << goal;

	pdef->setStartAndGoalStates(start, goal);

	// apply configurations to planner
	planner->setProblemDefinition(pdef);
	planner->setup();

//	// print the settings for this space
//	std::cout << "----------------------" << std::endl;
//	si->printSettings(std::cout);
//
//	// print the problem settings
//	std::cout << "----------------------" << std::endl;
//	pdef->print(std::cout);

	// attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = planner->solve(60.0);
	if (solved)
	{
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution:" << std::endl;

		// print the path to screen
		path->print(std::cout);
	}
	else
		std::cout << "No solution found" << std::endl;

	planner->clear();
}



