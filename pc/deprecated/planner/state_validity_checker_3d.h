/*
 * state_vadility_checker_2d.h
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PLANNER_STATE_VALIDITY_CHECKER_2D_H_
#define PLANNING_SRC_PLANNER_STATE_VALIDITY_CHECKER_2D_H_

#include <string>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "map/map_info.h"
#include "map/image_utils.h"
#include "map/map_utils.h"

namespace srcl_ctrl {

class StateValidityChecker3D : public ompl::base::StateValidityChecker {
public:
	StateValidityChecker3D(const ompl::base::SpaceInformationPtr &si) :
       ompl::base::StateValidityChecker(si),
	   tree_(new octomap::OcTree(0.1)),
	   octomap_ready_(false)
	{
		LoadOctomap("/home/rdu/Workspace/srcl_rtk/srcl_ctrl/build/bin/test_tree.bt");
	}

public:
	octomap::OcTree* tree_;
	MapInfo map_info_;
	bool octomap_ready_;
	double mmin_[3];
	double mmax_[3];

public:
	void LoadOctomap(std::string path)
	{
		tree_->readBinary(path);

		tree_->getMetricMin(mmin_[0],mmin_[1],mmin_[2]);
		tree_->getMetricMax(mmax_[0],mmax_[1],mmax_[2]);

		octomap_ready_ = true;
	}

	virtual bool isValid(const ompl::base::State *state) const
	{
		if(octomap_ready_)
		{
			octomap::point3d query(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
					state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
					state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);

			octomap::OcTreeNode* result = tree_->search (query);

			if (result != NULL) {
				//std::cout << "occupancy probability at " << query << ":\t " << result->getOccupancy() << std::endl;
				if(result->getOccupancy() < 0.9 )
					return true;
				else
					return false;
			}
			else
			{
//				std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
				return false;
			}
		}
		else
			return true;
	}
};

}

#endif /* PLANNING_SRC_PLANNER_STATE_VALIDITY_CHECKER_2D_H_ */
