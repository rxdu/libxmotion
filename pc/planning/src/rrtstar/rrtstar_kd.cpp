/*
 * rrtstar_kd.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#include "rrtstar_kd.h"

using namespace ompl;
using namespace srcl_ctrl;

RRTStarKD::RRTStarKD(const base::SpaceInformationPtr &si) : base::Planner(si, "RRT_Star_Kinodynamic")
{

}

RRTStarKD::~RRTStarKD()
{

}

base::PlannerStatus RRTStarKD::solve(const base::PlannerTerminationCondition &ptc)
{
	// make sure the planner is configured correctly; ompl::base::Planner::checkValidity
	// ensures that there is at least one input state and a ompl::base::Goal object specified
	checkValidity();

	base::Goal *goal   = pdef_->getGoal().get();
}

void RRTStarKD::setup()
{
	Planner::setup();
}

void RRTStarKD::clear()
{
	Planner::clear();
}

void RRTStarKD::getPlannerData(base::PlannerData &data) const
{

}


