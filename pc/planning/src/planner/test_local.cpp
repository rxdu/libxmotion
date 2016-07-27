/*
 * test_local.cpp
 *
 *  Created on: Jul 27, 2016
 *      Author: rdu
 */

#include "planner/local_planner.h"

using namespace srcl_ctrl;

int main(int argc, char** argv)
{
	LocalPlanner planner;

	planner.ConfigLocalPlanner();
	planner.SearchSolution();
}



