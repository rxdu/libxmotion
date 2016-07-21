/*
 * rrtstar_kd.h
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_RRTSTAR_RRTSTAR_KD_H_
#define PLANNING_SRC_RRTSTAR_RRTSTAR_KD_H_

#include <ompl/base/Planner.h>

#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>

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

	// other functions used to coordinate with ompl


	/******************* RRT Star Algorithm *******************/
public:



};

}

}


#endif /* PLANNING_SRC_RRTSTAR_RRTSTAR_KD_H_ */
