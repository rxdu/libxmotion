/*
 * rrtstar_kd.h
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_RRTSTAR_RRTSTAR_KD_H_
#define PLANNING_SRC_RRTSTAR_RRTSTAR_KD_H_

#include <cstdint>
#include <vector>

#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>

#include "rrtstar/motion.h"

namespace ompl {

namespace srcl_ctrl {

class RRTStarKD : public base::Planner
{
public:
	RRTStarKD(const base::SpaceInformationPtr &si);
	~RRTStarKD();

	/******************* OMPL Infrastructure *******************/
public:
	// strictly required functions by ompl
	base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

	// optional but recommended function by ompl
	void setup();
	void clear();
	void getPlannerData(base::PlannerData &data) const;


	/******************* RRT Star Algorithm *******************/
public:
	// A nearest-neighbors datastructure containing the tree of motions
	std::shared_ptr< NearestNeighbors<Motion*> > nn_;

	// sampling related
	base::StateSamplerPtr sampler_;
	base::OptimizationObjectivePtr opt_;

	// Stores the start states as Motions.
	std::vector<Motion*> startMotions_;
	// The most recent goal motion.  Used for PlannerData computation
	Motion *lastGoalMotion_;
	// Number of iterations the algorithm performed
	uint64_t iterations_;

	// search parameters
	// The maximum length of a motion to be added to a tree
	double maxDistance_;
	// Option to use k-nearest search for rewiring
	bool useKNearest_;
	// A constant for k-nearest rewiring calculations
	double k_rrg_;
	// A constant for r-disc rewiring calculations
	double r_rrg_;
	// Option to delay and reduce collision checking within iterations
	bool delayCC_;

protected:
	void getNeighbors(Motion *motion, std::vector<Motion*> &nbh) const;

	// For sorting a list of costs and getting only their sorted indices
	struct CostIndexCompare
	{
		CostIndexCompare(const std::vector<base::Cost>& costs,
				const base::OptimizationObjective &opt) :
					costs_(costs), opt_(opt) {}

		bool operator()(unsigned i, unsigned j)
		{
			return opt_.isCostBetterThan(costs_[i],costs_[j]);
		}

		const std::vector<base::Cost>& costs_;
		const base::OptimizationObjective &opt_;
	};

};

}

}


#endif /* PLANNING_SRC_RRTSTAR_RRTSTAR_KD_H_ */
