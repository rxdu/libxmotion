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
	void PostProcess2DPath(const std::vector<ompl::base::State*>& path, std::vector<Position2Dd>& waypoints);

private:
	void ConstructFlatOutputSpace();
	void Construct2DStateSpace();
	void DefinePlanProblem();
	void InitPlanner();

public:
	void ConfigLocalPlanner();
	bool SearchSolution();

	void SetStartAndGoal(Position2Dd start, Position2Dd goal);
	bool SearchSolution(Position2Dd start, Position2Dd goal, std::vector<Position2Dd>& path2d);
};

}

#endif /* PLANNING_SRC_PLANNER_RRTS_PLANNER_H_ */
