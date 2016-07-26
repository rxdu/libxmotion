/*
 * rrtstar_kd.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#include <algorithm>

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

#include <boost/math/constants/constants.hpp>

#include "rrtstar_kd.h"

using namespace ompl;
using namespace ompl::geometric;
using namespace srcl_ctrl;

RRTStarKD::RRTStarKD(const base::SpaceInformationPtr &si) :
				base::Planner(si, "RRT_Star_Kinodynamic"),
				lastGoalMotion_(nullptr),
				iterations_(0),
				maxDistance_(0.0),
				rewireFactor_(1.1),
				k_rrg_(0u),
				delayCC_(true),
				send_iteration_data(false)
{
	sampler_ = si_->allocStateSampler();
}

RRTStarKD::~RRTStarKD()
{

}

void RRTStarKD::setup()
{
	Planner::setup();

	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);

	if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
	{
		OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
	}

	if (!motion_tree_)
		motion_tree_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	motion_tree_->setDistanceFunction(std::bind(&RRTStarKD::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));

	// Setup optimization objective
	//
	// If no optimization objective was specified, then default to
	// optimizing path length as computed by the distance() function
	// in the state space.
	if (pdef_)
	{
		if (pdef_->hasOptimizationObjective())
			opt_ = pdef_->getOptimizationObjective();
		else
		{
			OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
			opt_.reset(new base::PathLengthOptimizationObjective(si_));

			// Store the new objective in the problem def'n
			pdef_->setOptimizationObjective(opt_);
		}
	}
	else
	{
		OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
		setup_ = false;
	}

	// Calculate some constants:
	calculateRewiringLowerBounds();

	// Set the bestCost_ and prunedCost_ as infinite
	bestCost_ = opt_->infiniteCost();
}

void RRTStarKD::clear()
{
	setup_ = false;
	Planner::clear();

	sampler_.reset();
	freeMemory();
	if (motion_tree_)
		motion_tree_->clear();

	lastGoalMotion_ = nullptr;
	startMotions_.clear();
	goalMotions_.clear();

	iterations_ = 0;
	bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void RRTStarKD::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	std::vector<Motion*> motions;
	if (motion_tree_)
		motion_tree_->list(motions);

	if (lastGoalMotion_)
		data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

	for (std::size_t i = 0 ; i < motions.size() ; ++i)
	{
		if (motions[i]->parent == nullptr)
			data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
		else
			data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
					base::PlannerDataVertex(motions[i]->state));
	}
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
	base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

	bool symCost = opt_->isSymmetric();

	// Check if there are more starts
	if (pis_.haveMoreStartStates() == true)
	{
		// There are, add them
		while (const base::State *st = pis_.nextStart())
		{
			Motion *motion = new Motion(si_);
			si_->copyState(motion->state, st);
			motion->cost = opt_->identityCost();
			motion_tree_->add(motion);
			startMotions_.push_back(motion);
		}
	}

	// the initial motion tree should contain at least one start state
	if (motion_tree_->size() == 0)
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	// just make sure a sampler is allocated
	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), motion_tree_->size());
	OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(), (unsigned int)std::ceil(k_rrg_ * log((double)(motion_tree_->size() + 1u))));

	const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

	Motion *solution = lastGoalMotion_;
	Motion *approx_solution = nullptr;
	double approx_dist = std::numeric_limits<double>::infinity();
	bool sufficientlyShort = false;

	Motion *sampled_motion = new Motion(si_);
	base::State *sampled_state = sampled_motion->state;
	base::State *interpolated_state = si_->allocState();

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
		if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && goal_s->canSample())
			goal_s->sampleGoal(sampled_state);
		else
			sampler_->sampleUniform(sampled_state);

		// find closest state in the tree
		Motion *nearest_motion = motion_tree_->nearest(sampled_motion);

		if (intermediateSolutionCallback && si_->equalStates(nearest_motion->state, sampled_state))
			continue;

		base::State *new_state = sampled_state;

		// find state to add to the tree
		double d = si_->distance(nearest_motion->state, sampled_state);
		if (d > maxDistance_)
		{
			si_->getStateSpace()->interpolate(nearest_motion->state, sampled_state, maxDistance_ / d, interpolated_state);
			new_state = interpolated_state;
		}

		// Check if the motion between the nearest state and the state to add is valid
		if (si_->checkMotion(nearest_motion->state, new_state))
		{
			// create a motion
			Motion *new_motion = new Motion(si_);
			si_->copyState(new_motion->state, new_state);
			new_motion->parent = nearest_motion;
			new_motion->incCost = opt_->motionCost(nearest_motion->state, new_motion->state);
			new_motion->cost = opt_->combineCosts(nearest_motion->cost, new_motion->incCost);

			// Find nearby neighbors of the new motion
			getNeighbors(new_motion, nbh);

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
					incCosts[i] = opt_->motionCost(nbh[i]->state, new_motion->state);
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
					if (nbh[*i] == nearest_motion || si_->checkMotion(nbh[*i]->state, new_motion->state))
					{
						new_motion->incCost = incCosts[*i];
						new_motion->cost = costs[*i];
						new_motion->parent = nbh[*i];
						valid[*i] = 1;
						break;
					}
					else valid[*i] = -1;
				}
			}
			else
			{
				// Finding the nearest neighbor to connect to
				new_motion->incCost = opt_->motionCost(nearest_motion->state, new_motion->state);
				new_motion->cost = opt_->combineCosts(nearest_motion->cost, new_motion->incCost);

				// find which one we connect the new state to
				// evaluate each neighbour to find the one resulting in better cost
				for (std::size_t i = 0 ; i < nbh.size(); ++i)
				{
					if (nbh[i] != nearest_motion)
					{
						incCosts[i] = opt_->motionCost(nbh[i]->state, new_motion->state);
						costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
						if (opt_->isCostBetterThan(costs[i], new_motion->cost))
						{
							if (si_->checkMotion(nbh[i]->state, new_motion->state))
							{
								new_motion->incCost = incCosts[i];
								new_motion->cost = costs[i];
								new_motion->parent = nbh[i];
								valid[i] = 1;
							}
							else valid[i] = -1;
						}
					}
					else
					{
						incCosts[i] = new_motion->incCost;
						costs[i] = new_motion->cost;
						valid[i] = 1;
					}
				}
			}

//			std::cout << "sampled state: "
//				<< new_motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]
//				<< " , "
//				<< new_motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]
//				<< std::endl;

			// add motion to the tree
			motion_tree_->add(new_motion);
			new_motion->parent->children.push_back(new_motion);

			// rewire the tree
			bool checkForSolution = false;
			for (std::size_t i = 0; i < nbh.size(); ++i)
			{
				if (nbh[i] != new_motion->parent)
				{
					base::Cost nbhIncCost;
					if (symCost)
						nbhIncCost = incCosts[i];
					else
						nbhIncCost = opt_->motionCost(new_motion->state, nbh[i]->state);
					base::Cost nbhNewCost = opt_->combineCosts(new_motion->cost, nbhIncCost);
					if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
					{
						bool motionValid;
						if (valid[i] == 0)
						{
							motionValid = si_->checkMotion(new_motion->state, nbh[i]->state);
						}
						else
						{
							motionValid = (valid[i] == 1);
						}

						if (motionValid)
						{
							// Remove this node from its parent list
							removeFromParent (nbh[i]);

							// Add this node to the new parent
							nbh[i]->parent = new_motion;
							nbh[i]->incCost = nbhIncCost;
							nbh[i]->cost = nbhNewCost;
							nbh[i]->parent->children.push_back(nbh[i]);

							// Update the costs of the node's children
							updateChildCosts(nbh[i]);

							checkForSolution = true;
						}
					}
				}
			}

			// Add the new motion to the goalMotion_ list, if it satisfies the goal
			double distanceFromGoal;
			if (goal->isSatisfied(new_motion->state, &distanceFromGoal))
			{
				goalMotions_.push_back(new_motion);
				checkForSolution = true;
			}

			// Checking for solution or iterative improvement
			if (checkForSolution)
			{
				bool updatedSolution = false;
				for (size_t i = 0; i < goalMotions_.size(); ++i)
				{
					if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost_))
					{
						if (opt_->isFinite(bestCost_) == false)
						{
							OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u vertices in the graph)", getName().c_str(), goalMotions_[i]->cost.value(), iterations_, motion_tree_->size());
						}
						bestCost_ = goalMotions_[i]->cost;
						updatedSolution = true;
					}

					sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->cost);
					if (sufficientlyShort)
					{
						solution = goalMotions_[i];
						break;
					}
					else if (!solution ||
							opt_->isCostBetterThan(goalMotions_[i]->cost,solution->cost))
					{
						solution = goalMotions_[i];
						updatedSolution = true;
					}
				}

				if (updatedSolution || send_iteration_data)
				{
					if (intermediateSolutionCallback)
					{
						std::vector<const base::State *> spath;
						Motion *intermediate_solution = solution->parent; // Do not include goal state to simplify code.

						//Push back until we find the start, but not the start itself
						while (intermediate_solution->parent != nullptr)
						{
							spath.push_back(intermediate_solution->state);
							intermediate_solution = intermediate_solution->parent;
						}

						intermediateSolutionCallback(this, spath, bestCost_);
					}
				}
			}

			// Checking for approximate solution (closest state found to the goal)
			if (goalMotions_.size() == 0 && distanceFromGoal < approx_dist)
			{
				approx_solution = new_motion;
				approx_dist = distanceFromGoal;
			}
		}

		// terminate if a sufficient solution is found
		if (solution && sufficientlyShort)
			break;
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
		if (is_approximate)
			psol.setApproximate(approx_dist);
		// Does the solution satisfy the optimization objective?
		psol.setOptimized(opt_, bestCost_, sufficientlyShort);
		pdef_->addSolutionPath(psol);

		found_solution = true;
	}

	// cleanup memory
	si_->freeState(interpolated_state);
	if (sampled_motion->state)
		si_->freeState(sampled_motion->state);
	delete sampled_motion;

	OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost %.3f", getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

	return base::PlannerStatus(found_solution, is_approximate);
}

void RRTStarKD::getNeighbors(Motion *motion, std::vector<Motion*> &nbh) const
{
	double cardDbl = static_cast<double>(motion_tree_->size() + 1u);

	//- k-nearest RRT*
	unsigned int k = std::ceil(k_rrg_ * log(cardDbl));
	motion_tree_->nearestK(motion, k, nbh);
}

void RRTStarKD::removeFromParent(Motion *m)
{
	for (std::vector<Motion*>::iterator it = m->parent->children.begin ();
			it != m->parent->children.end (); ++it)
	{
		if (*it == m)
		{
			m->parent->children.erase(it);
			break;
		}
	}
}

void RRTStarKD::updateChildCosts(Motion *m)
{
	for (std::size_t i = 0; i < m->children.size(); ++i)
	{
		m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
		updateChildCosts(m->children[i]);
	}
}

void RRTStarKD::calculateRewiringLowerBounds()
{
	double dimDbl = static_cast<double>(si_->getStateDimension());

	// k_rrg > e+e/d.  K-nearest RRT*
	k_rrg_ = rewireFactor_ * (boost::math::constants::e<double>() + (boost::math::constants::e<double>() / dimDbl));
}

void RRTStarKD::freeMemory()
{
	if (motion_tree_)
	{
		std::vector<Motion*> motions;
		motion_tree_->list(motions);
		for (std::size_t i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}
}
