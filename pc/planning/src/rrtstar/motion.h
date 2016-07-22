/*
 * motion.h
 *
 *  Created on: Jul 22, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_RRTSTAR_MOTION_H_
#define PLANNING_SRC_RRTSTAR_MOTION_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>

namespace ompl {
namespace srcl_ctrl {

class Motion
{
public:
	/** \brief Constructor that allocates memory for the state. This constructor automatically allocates memory for \e state, \e cost, and \e incCost */
	Motion(const base::SpaceInformationPtr &si) :
		state(si->allocState()),
		parent(nullptr){}

	~Motion()
	{
	}

	/** \brief The state contained by the motion */
	base::State       *state;

	/** \brief The parent motion in the exploration tree */
	Motion            *parent;

	/** \brief The cost up to this motion */
	base::Cost        cost;

	/** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
	base::Cost        incCost;

	/** \brief The set of motions descending from the current motion */
	std::vector<Motion*> children;
};

}
}

#endif /* PLANNING_SRC_RRTSTAR_MOTION_H_ */
