/*
 * =====================================================================================
 *
 *       Filename:  test_smp.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  06/27/2016 05:04:05 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ruixiang Du (rdu), ruixiang.du@gmail.com
 *   Organization:  Worcester Polytechnic Institute
 *
 * =====================================================================================
 */

#include <stdlib.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;

void SetupPlanning(void)
{
	// construct the flat output space of the quadrotor
	ompl::base::StateSpacePtr r3(new ompl::base::RealVectorStateSpace(3));
	ompl::base::RealVectorBounds bounds(3);
	bounds.setLow(-1);
	bounds.setHigh(1);
	r3->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());
	ompl::base::StateSpacePtr flat_space = r3 + so2;

	ompl::base::ScopedState<> state(flat_space);
	std::cout << state;
}

int main(int argc, char *argv[]) {
	SetupPlanning();

    return EXIT_SUCCESS;
}
