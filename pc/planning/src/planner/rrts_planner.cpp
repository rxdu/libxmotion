/*
 * rrts_planner.cpp
 *
 *  Created on: Jul 27, 2016
 *      Author: rdu
 */

#include <iostream>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <rrts_planner.h>

using namespace ompl;
using namespace srcl_ctrl;

namespace ob = ompl::base;
namespace og = ompl::geometric;

RRTStarPlanner::RRTStarPlanner():
		planner_ready_(false),
		state_space_(nullptr),
		space_info_(nullptr)
{

}

RRTStarPlanner::~RRTStarPlanner()
{


}

std::vector<Position2Dd> RRTStarPlanner::PostProcess2DPath(std::vector<ompl::base::State*> path)
{
	std::vector<Position2Dd> waypoints;

	for(auto& wp : path)
	{
		Position2Dd pos;
		pos.x = wp->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
		pos.y = wp->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
		waypoints.push_back(pos);
	}

	std::cout << "2d path: " << std::endl;
	for(auto& pt : waypoints)
	{
		std::cout << pt.x << " , " << pt.y << std::endl;
	}

	return waypoints;
}

void RRTStarPlanner::ConstructFlatOutputSpace()
{
	ompl::base::StateSpacePtr r3(new ompl::base::RealVectorStateSpace(3));
	ompl::base::RealVectorBounds bounds(3);
	bounds.setLow(-10);
	bounds.setHigh(10);
	r3->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	// define the SO(2) space for (yaw)
	ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());

	state_space_ = r3 + so2;

	space_info_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
}

void RRTStarPlanner::Construct2DStateSpace()
{
	// define the R2 space for testing (x,y)
	ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));
	ompl::base::RealVectorBounds bounds2(2);
	bounds2.setLow(0);
	bounds2.setHigh(1.5);
	r2->as<ob::RealVectorStateSpace>()->setBounds(bounds2);

	state_space_ = r2;

	space_info_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
}

void RRTStarPlanner::DefinePlanProblem()
{
	problem_def_ = std::make_shared<ob::ProblemDefinition>(space_info_);

	//problem_def_->setIntermediateSolutionCallback(test_callback);

	ompl::base::ScopedState<> start(state_space_);
	start.random();
	//	start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
	//	start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;
	ob::ScopedState<> goal(state_space_);
	goal.random();
	//	goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
	//	goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

	std::cout << "\n Start: " << std::endl;
	std::cout << start;

	std::cout << "\n Goal: " << std::endl;
	std::cout << goal;

	problem_def_->setStartAndGoalStates(start, goal);
}

void RRTStarPlanner::InitPlanner()
{
	auto rrt_planner =  new RRTStarKD(space_info_);
	rrt_planner->setRange(0.01);

	// auto rrt_planner =  new og::RRTstar(si);

	planner_ = ob::PlannerPtr(rrt_planner);

	planner_->setProblemDefinition(problem_def_);
	planner_->setup();
}

void RRTStarPlanner::ConfigLocalPlanner()
{
	//ConstructFlatOutputSpace();
	Construct2DStateSpace();
	DefinePlanProblem();
	InitPlanner();

	planner_ready_ = true;
}

bool RRTStarPlanner::SearchSolution()
{
	if(!planner_ready_) {
		std::cerr << "You need to call ConfigLocalPlanner() to setup the local planner first." << std::endl;
		return false;
	}

	ob::PlannerStatus solved = planner_->solve(1.5);

	if (solved)
	{
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		ob::PathPtr path = problem_def_->getSolutionPath();

		if(solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
			std::cout << "Found approximate solution" << std::endl;
		else if(solved == ob::PlannerStatus::EXACT_SOLUTION)
			std::cout << "Found exact solution" << std::endl;

		// print the path to screen
		path->print(std::cout);

		PostProcess2DPath(path->as<og::PathGeometric>()->getStates());

		//		std::ofstream outFile("output.txt");
		//		dynamic_cast<const og::PathGeometric&>(*path).printAsMatrix(std::cout);
		//		path->as<og::PathGeometric>()->printAsMatrix(outFile);
	}
	else
		std::cout << "No solution found" << std::endl;

	if(solved == ob::PlannerStatus::APPROXIMATE_SOLUTION ||
			solved == ob::PlannerStatus::EXACT_SOLUTION)
		return true;
	else
		return false;

}
