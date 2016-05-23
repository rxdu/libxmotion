/*
 * motion_server.cpp
 *
 *  Created on: May 23, 2016
 *      Author: rdu
 */

#include "motion_server/motion_server.h"

using namespace srcl_ctrl;

MotionServer::MotionServer()
{

}

MotionServer::~MotionServer()
{

}

void MotionServer::SetMotionGoal(std::vector<TrajectoryPoint>& goal)
{

}

void MotionServer::AbortActiveMotion()
{
	active_goal_.clear();
}

double MotionServer::GetActiveMotionProgress()
{

}
