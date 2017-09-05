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

#include "opencv2/opencv.hpp"

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>

#include "common/planning_types.h"
#include "planner/state_validity_checker_3d.h"
#include "rrtstar/rrtstar_kd.h"
#include "rrtstar/rrt_node.h"
#include "map/map_info.h"
#include "graph/graph.h"

namespace srcl_ctrl {

class RRTStarPlannerLocal {
public:
	RRTStarPlannerLocal();
	~RRTStarPlannerLocal();

private:
	bool planner_ready_;
	ompl::base::StateSpacePtr state_space_;
	ompl::base::SpaceInformationPtr space_info_;
	ompl::base::PlannerPtr planner_;
	ompl::base::ProblemDefinitionPtr problem_def_;

public:
	StateValidityChecker3D* validity_checker_3d_;
	std::shared_ptr<Graph_t<RRTNode>> rrts_vis_graph_;

private:
	void SetStartAndGoal(Position3Dd start, Position3Dd goal);
	void RRTStatusCallback(const ompl::base::Planner* planner, const std::vector<const ompl::base::State*> & states, const ompl::base::Cost cost);
	void ProcessPlannerData(ompl::base::PlannerData& data);
	void PostProcess3DPath(const std::vector<ompl::base::State*>& path, std::vector<Position3Dd>& waypoints);

private:
	void ConstructFlatOutputSpace();
	void Construct3DStateSpace();
	void Set3DStateSpaceBound(double xmin, double xmax, double ymin, double ymax);
	void DefinePlanProblem();
	void InitPlanner();
	void ConfigLocalPlanner();

public:
	void UpdateOccupancyMap(cv::Mat map, MapInfo info);
	bool SearchSolution();

	bool SearchSolution(Position3Dd start, Position3Dd goal, double time_limit, std::vector<Position3Dd>& path3d);
};

}

#endif /* PLANNING_SRC_PLANNER_RRTS_PLANNER_H_ */
