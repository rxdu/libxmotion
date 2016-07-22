/*
 * rrtstar_kd.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#include <algorithm>

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

#include "rrtstar_kd.h"

using namespace ompl;
using namespace ompl::geometric;
using namespace srcl_ctrl;

RRTStarKD::RRTStarKD(const base::SpaceInformationPtr &si) :
		base::Planner(si, "RRT_Star_Kinodynamic"),
		lastGoalMotion_(nullptr),
		iterations_(0),
		maxDistance_(0.01),
		useKNearest_(true),
		k_rrg_(0u),
		r_rrg_(0.0),
		delayCC_(true)
{
	sampler_ = si_->allocStateSampler();
}

RRTStarKD::~RRTStarKD()
{

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

base::PlannerStatus RRTStarKD::solve(const base::PlannerTerminationCondition &ptc)
{
	// make sure the planner is configured correctly; ompl::base::Planner::checkValidity
	// ensures that there is at least one input state and a ompl::base::Goal object specified
	checkValidity();

	/*----------------------------------------------------------*/
	/*							pre-search						*/
	/*----------------------------------------------------------*/
	base::Goal *goal = pdef_->getGoal().get();
	base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

	// Check if there are more starts
	if (pis_.haveMoreStartStates() == true)
	{
		// There are, add them
		while (const base::State *st = pis_.nextStart())
		{
			Motion *motion = new Motion(si_);
			si_->copyState(motion->state, st);
			motion->cost = opt_->identityCost();
			nn_->add(motion);
			startMotions_.push_back(motion);
		}
	}

	if (nn_->size() == 0)
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	// just make sure a sampler is allocated
	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

	const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

	Motion *solution = nullptr; //lastGoalMotion_;
	Motion *approx_solution = nullptr;

	double approx_dist = std::numeric_limits<double>::infinity();

	Motion *rmotion = new Motion(si_);
	base::State *rstate = rmotion->state;
	base::State *xstate = si_->allocState();

	std::vector<Motion*> nbh;
	std::vector<int> valid;
	uint64_t rewireTest = 0;
	uint64_t statesGenerated = 0;

	std::vector<base::Cost> costs;
	std::vector<base::Cost> incCosts;
	std::vector<std::size_t> sortedCostIndices;

	// our functor for sorting nearest neighbors
	CostIndexCompare compareFn(costs, *opt_);

	/*----------------------------------------------------------*/
	/*						search process						*/
	/*----------------------------------------------------------*/
	while (ptc == false)
	{
		iterations_++;

		// sample a state
		sampler_->sampleUniform(rstate);

		// find closest state in the tree
		Motion *nmotion = nn_->nearest(rmotion);

		base::State *dstate = rstate;
		// find state to add to the tree
		double d = si_->distance(nmotion->state, rstate);
		if (d > maxDistance_)
		{
			si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
			dstate = xstate;
		}

		// Check if the motion between the nearest state and the state to add is valid
		if (si_->checkMotion(nmotion->state, dstate))
		{
			// create a motion
			Motion *motion = new Motion(si_);
			si_->copyState(motion->state, dstate);
			motion->parent = nmotion;
			motion->incCost = opt_->motionCost(nmotion->state, motion->state);
			motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

			// Find nearby neighbors of the new motion
			getNeighbors(motion, nbh);

			rewireTest += nbh.size();
			++statesGenerated;

			// cache for distance computations
			//
			// Our cost caches only increase in size, so they're only
			// resized if they can't fit the current neighborhood
			if (costs.size() < nbh.size())
			{
				costs.resize(nbh.size());
				incCosts.resize(nbh.size());
				sortedCostIndices.resize(nbh.size());
			}

			// cache for motion validity (only useful in a symmetric space)
			//
			// Our validity caches only increase in size, so they're
			// only resized if they can't fit the current neighborhood
			if (valid.size() < nbh.size())
				valid.resize(nbh.size());
			std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

			// Finding the nearest neighbor to connect to
			// By default, neighborhood states are sorted by cost, and collision checking
			// is performed in increasing order of cost
			if (delayCC_)
			{
				// calculate all costs and distances
				for (std::size_t i = 0 ; i < nbh.size(); ++i)
				{
					incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
					costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
				}

				// sort the nodes
				//
				// we're using index-value pairs so that we can get at
				// original, unsorted indices
				for (std::size_t i = 0; i < nbh.size(); ++i)
					sortedCostIndices[i] = i;
				std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(),
						compareFn);

				// collision check until a valid motion is found
				//
				// ASYMMETRIC CASE: it's possible that none of these
				// neighbors are valid. This is fine, because motion
				// already has a connection to the tree through
				// nmotion (with populated cost fields!).
				for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
						i != sortedCostIndices.begin() + nbh.size();
						++i)
				{
					if (nbh[*i] == nmotion || si_->checkMotion(nbh[*i]->state, motion->state))
					{
						motion->incCost = incCosts[*i];
						motion->cost = costs[*i];
						motion->parent = nbh[*i];
						valid[*i] = 1;
						break;
					}
					else valid[*i] = -1;
				}
			}
			else // if not delayCC
			{
				motion->incCost = opt_->motionCost(nmotion->state, motion->state);
				motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
				// find which one we connect the new state to
				for (std::size_t i = 0 ; i < nbh.size(); ++i)
				{
					if (nbh[i] != nmotion)
					{
						incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
						costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
						if (opt_->isCostBetterThan(costs[i], motion->cost))
						{
							if (si_->checkMotion(nbh[i]->state, motion->state))
							{
								motion->incCost = incCosts[i];
								motion->cost = costs[i];
								motion->parent = nbh[i];
								valid[i] = 1;
							}
							else valid[i] = -1;
						}
					}
					else
					{
						incCosts[i] = motion->incCost;
						costs[i] = motion->cost;
						valid[i] = 1;
					}
				}
			}
		}
	}


	/*----------------------------------------------------------*/
	/* 						post - search						*/
	/*----------------------------------------------------------*/
	bool is_approximate = (solution == nullptr);
	bool found_solution = false;
	if (is_approximate)
		solution = approx_solution;
	else
		lastGoalMotion_ = solution;

	if (solution != nullptr)
	{
		ptc.terminate();
		// construct the solution path
		std::vector<Motion*> mpath;
		while (solution != nullptr)
		{
			mpath.push_back(solution);
			solution = solution->parent;
		}

		// set the solution path
		PathGeometric *geoPath = new PathGeometric(si_);
		for (int i = mpath.size() - 1 ; i >= 0 ; --i)
			geoPath->append(mpath[i]->state);

		base::PathPtr path(geoPath);
		// Add the solution path.
		base::PlannerSolution psol(path);
		psol.setPlannerName(getName());
//		if (approximate)
//			psol.setApproximate(approximatedist);
//		// Does the solution satisfy the optimization objective?
//		psol.setOptimized(opt_, bestCost_, sufficientlyShort);
		pdef_->addSolutionPath(psol);

		found_solution = true;
	}

	// cleanup memory


	//OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost %.3f", getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

	return base::PlannerStatus(found_solution, is_approximate);
}

void RRTStarKD::getNeighbors(Motion *motion, std::vector<Motion*> &nbh) const
{
    double cardDbl = static_cast<double>(nn_->size() + 1u);
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrg_ * log(cardDbl));
        nn_->nearestK(motion, k, nbh);
    }
    else
    {
        double r = std::min(maxDistance_, r_rrg_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
        nn_->nearestR(motion, r, nbh);
    }
}
