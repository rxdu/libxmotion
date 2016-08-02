/*
 * rrts_planner.h
 *
 *  Created on: Jul 27, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PLANNER_RRTS_PLANNER_H_
#define PLANNING_SRC_PLANNER_RRTS_PLANNER_H_

#include <vector>
#include <memory>

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>

#include "rrtstar/rrtstar_kd.h"
#include "common/planning_types.h"

namespace srcl_ctrl {

class RRTStarPlanner {
public:
	RRTStarPlanner();
	~RRTStarPlanner();

private:
	bool planner_ready_;
	ompl::base::StateSpacePtr state_space_;
	ompl::base::SpaceInformationPtr space_info_;
	ompl::base::PlannerPtr planner_;
	ompl::base::ProblemDefinitionPtr problem_def_;

private:
	std::vector<Position2Dd> PostProcess2DPath(std::vector<ompl::base::State*> path);

private:
	void ConstructFlatOutputSpace();
	void Construct2DStateSpace();
	void DefinePlanProblem();
	void InitPlanner();

public:
	void ConfigLocalPlanner();
	bool SearchSolution();
};

}

#endif /* PLANNING_SRC_PLANNER_RRTS_PLANNER_H_ */
