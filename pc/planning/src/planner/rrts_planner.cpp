/*
 * rrts_planner.cpp
 *
 *  Created on: Jul 27, 2016
 *      Author: rdu
 */

#include <iostream>
#include <fstream>
#include <functional>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/ScopedState.h>

#include <rrts_planner.h>

using namespace ompl;
using namespace srcl_ctrl;

namespace ob = ompl::base;
namespace og = ompl::geometric;

RRTStarPlanner::RRTStarPlanner():
		planner_ready_(false),
		state_space_(nullptr),
		space_info_(nullptr),
		validity_checker_2d_(nullptr)
{
	this->ConfigLocalPlanner();
}

RRTStarPlanner::~RRTStarPlanner()
{


}

void RRTStarPlanner::PostProcess2DPath(const std::vector<ompl::base::State*>& path, std::vector<Position2Dd>& waypoints)
{
	for(auto& wp : path)
	{
		Position2Dd pos;
		pos.x = wp->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
		pos.y = wp->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
		waypoints.push_back(pos);
	}

//	std::cout << "2d path: " << std::endl;
//	for(auto& pt : waypoints)
//	{
//		std::cout << pt.x << " , " << pt.y << std::endl;
//	}
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
	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(0,0);
	bounds.setHigh(0,10.0);
	bounds.setLow(1,0);
	bounds.setHigh(1,5.0);
	r2->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	state_space_ = r2;

	space_info_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);

	// set state validity checker
	validity_checker_2d_ = new StateValidityChecker2D(space_info_);
	space_info_->setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker_2d_));
}

void RRTStarPlanner::Set2DStateSpaceBound(double xmin, double xmax, double ymin, double ymax)
{
	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(0,xmin);
	bounds.setHigh(0,xmax);
	bounds.setLow(1,ymin);
	bounds.setHigh(1,ymax);

	state_space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);
}

void RRTStarPlanner::DefinePlanProblem()
{
	problem_def_ = std::make_shared<ob::ProblemDefinition>(space_info_);

	problem_def_->setIntermediateSolutionCallback(std::bind(&RRTStarPlanner::RRTStatusCallback, this,
			   std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void RRTStarPlanner::InitPlanner()
{
	auto rrt_planner =  new RRTStarKD(space_info_);
	rrt_planner->setRange(0.01);
	rrt_planner->EnableIterationData(true);

	// auto rrt_planner =  new og::RRTstar(si);

	planner_ = ob::PlannerPtr(rrt_planner);

	planner_->setProblemDefinition(problem_def_);
	planner_->setup();
}

void RRTStarPlanner::UpdateOccupancyMap(cv::Mat map, MapInfo info)
{
	validity_checker_2d_->SetOccupancyMap(map, info);

	//std::cout << "------------->>>>>>>>>>>>>>>>" << info.world_size_x << " , " << info.world_size_y << std::endl;

	Set2DStateSpaceBound(0,info.world_size_x, 0, info.world_size_y);
}

void RRTStarPlanner::ConfigLocalPlanner()
{
	//ConstructFlatOutputSpace();
	Construct2DStateSpace();
	DefinePlanProblem();
	InitPlanner();

	planner_ready_ = true;
}

void RRTStarPlanner::ProcessPlannerData(ompl::base::PlannerData& status_data)
{
	std::cout << "number of vertices: " << status_data.numVertices() << std::endl;

//	auto vtx = status_data.getVertex(0);
//
//	std::vector<unsigned int> edges;
//	status_data.getEdges(0, edges);
//
//	for(auto& edge:edges)
//		std::cout << "edge: " << edge << std::endl;

	rrts_vis_graph_ = std::make_shared<Graph_t<RRTNode>>();

	for(int i = 0; i < status_data.numVertices(); i++)
	{
		auto vtx = status_data.getVertex(i);

		RRTNode node(i);
		node.position.x = vtx.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
		node.position.y = vtx.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

		std::vector<unsigned int> edges;
		status_data.getEdges(i, edges);
		for(auto& edge : edges)
		{
			auto vtx2 = status_data.getVertex(edge);

			RRTNode node2(edge);
			node2.position.x = vtx2.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
			node2.position.y = vtx2.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

			rrts_vis_graph_->AddEdge(node, node2, 1);
		}
	}

//	for(int i = 0; i < status_data.numVertices(); i++)
//	{
//		auto vtx = status_data.getVertex(i);
//		if(status_data.isStartVertex(i))
//			std::cout << "*** " << " start vertex" << std::endl;
//		std::cout << i << " : " << vtx.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]
//						<< " , " << vtx.getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << std::endl;
//	}

	//std::ofstream outFile("graph_output.graphml");
	//status_data.printGraphviz(outFile);
	//status_data.printGraphML(outFile);
}

void RRTStarPlanner::RRTStatusCallback(const ompl::base::Planner* planner, const std::vector<const ompl::base::State*> & states, const ompl::base::Cost cost)
{
	std::cout << "callback called" << std::endl;
	ob::PlannerData status_data(planner->getSpaceInformation());
	planner->getPlannerData(status_data);

	ProcessPlannerData(status_data);
}

void RRTStarPlanner::SetStartAndGoal(Position2Dd start, Position2Dd goal)
{
	ompl::base::ScopedState<> start_state(state_space_);
	start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start.x;
	start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start.y;
	ob::ScopedState<> goal_state(state_space_);
	goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal.x;
	goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal.y;

	std::cout << " ------------------------------ " << std::endl;
	std::cout << "Start: " << std::endl;
	std::cout << start_state;

	std::cout << "\n Goal: " << std::endl;
	std::cout << goal_state;
	std::cout << " ------------------------------ " << std::endl;

	problem_def_->setStartAndGoalStates(start_state, goal_state);
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

		//PostProcess2DPath(path->as<og::PathGeometric>()->getStates());

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

bool RRTStarPlanner::SearchSolution(Position2Dd start, Position2Dd goal, double time_limit, std::vector<Position2Dd>& path2d)
{
	if(!planner_ready_) {
		std::cerr << "You need to call ConfigLocalPlanner() to setup the local planner first." << std::endl;
		return false;
	}

	SetStartAndGoal(start, goal);

	ob::PlannerStatus solved = planner_->solve(time_limit); // 1.5

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
		//path->print(std::cout);
		//std::ofstream outFile("output.txt");
		//path->as<og::PathGeometric>()->printAsMatrix(outFile);

		PostProcess2DPath(path->as<og::PathGeometric>()->getStates(), path2d);
	}
	else
		std::cout << "No solution found" << std::endl;

	if(solved == ob::PlannerStatus::APPROXIMATE_SOLUTION ||
			solved == ob::PlannerStatus::EXACT_SOLUTION)
		return true;
	else
		return false;
}
