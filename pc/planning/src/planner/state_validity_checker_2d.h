/*
 * state_vadility_checker_2d.h
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PLANNER_STATE_VALIDITY_CHECKER_2D_H_
#define PLANNING_SRC_PLANNER_STATE_VALIDITY_CHECKER_2D_H_

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace srcl_ctrl {

class StateValidityChecker2D : public ompl::base::StateValidityChecker {
public:
	StateValidityChecker2D(const ompl::base::SpaceInformationPtr &si) :
       ompl::base::StateValidityChecker(si) {}

     virtual bool isValid(const ompl::base::State *state) const
     {
    	 Position2Dd pos;

    	 pos.x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    	 pos.y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

    	 return true;
     }
};

}

#endif /* PLANNING_SRC_PLANNER_STATE_VALIDITY_CHECKER_2D_H_ */
