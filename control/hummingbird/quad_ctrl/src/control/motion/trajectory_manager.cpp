/*
 * trajectory_manager.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: rdu
 */

#include <cmath>

#include <motion/trajectory_manager.h>

using namespace srcl_ctrl;

TrajectoryManager::TrajectoryManager()
{
//	SetTestTrajectory();
	SetTestStraigtTrajectory();
}

TrajectoryManager::~TrajectoryManager()
{

}

void TrajectoryManager::ClearTrajectory()
{
	traj_.clear();
}

TrajectoryPoint TrajectoryManager::GetTrajectoryPoint(uint64_t t)
{
	TrajectoryPoint pt;
	pt.point_empty = true;

	if(!traj_.empty())
	{
		pt = *(traj_.begin());
		traj_.erase(traj_.begin());
	}

	return pt;
}

void TrajectoryManager::SetTrajectory(std::vector<TrajectoryPoint>& traj)
{
	traj_ = traj;
}

void TrajectoryManager::SetTestTrajectory()
{
	traj_.clear();

	for(int i = 0; i < 1000; i++)
	{
		TrajectoryPoint pt;
		pt.point_empty = false;

		double height = 0.5;
		double radius = 1.5;
		double circle_ang_vel = 180.0/180.0*3.14;

		if(i < 200)
		{
			pt.positions[0] = 0;
			pt.positions[1] = 0;
			pt.positions[2] = 0.5;
			pt.velocities[0] = 0;
			pt.velocities[1] = 0;
			pt.velocities[2] = 0;
			pt.accelerations[0] = 0;
			pt.accelerations[1] = 0;
			pt.accelerations[2] = 0;
			pt.yaw = 0;
		}
		else if(i < 600)
		{
			double angle = (i - 200)*0.01*circle_ang_vel;
			pt.positions[0] = radius * cos(angle - M_PI/2);
			pt.positions[1] = radius * sin(angle - M_PI/2);
			pt.positions[2] = height;

			pt.velocities[0] = - radius * sin(angle - M_PI/2) * 0.01*circle_ang_vel;
			pt.velocities[1] = radius * cos(angle - M_PI/2) * 0.01*circle_ang_vel;
			pt.velocities[2] = 0;

			pt.accelerations[0] = - radius * cos(angle - M_PI/2) * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
			pt.accelerations[1] = - radius * sin(angle - M_PI/2) * 0.01*circle_ang_vel * 0.01*circle_ang_vel;
			pt.accelerations[2] = 0;

			pt.yaw = angle;
		}
		else
			pt.point_empty = true;

		traj_.push_back(pt);
	}
}

void TrajectoryManager::SetTestStraigtTrajectory()
{
	traj_.clear();

	for(int i = 0; i < 1000; i++)
	{
		TrajectoryPoint pt;
		pt.point_empty = false;

		double height = 0.5;

		if(i < 200)
		{
			pt.positions[0] = 0;
			pt.positions[1] = 0;
			pt.positions[2] = 0.5;
			pt.velocities[0] = 0;
			pt.velocities[1] = 0;
			pt.velocities[2] = 0;
			pt.accelerations[0] = 0;
			pt.accelerations[1] = 0;
			pt.accelerations[2] = 0;
			pt.yaw = 0;
		}
		else if(i < 450)
		{
			double t = (i - 200) * 0.01;
			pt.positions[0] = -1/6.0*(t*t*t) + 2.5/4.0*t*t;
			pt.positions[1] = 0;
			pt.positions[2] = height;

			pt.velocities[0] = - 1/2.0*t*t + 2.5/2.0*t;
			pt.velocities[1] = 0;
			pt.velocities[2] = 0;

			pt.accelerations[0] = - t + 2.5/2.0;
			pt.accelerations[1] = 0;
			pt.accelerations[2] = 0;

			pt.yaw = 0;
		}
		else
			pt.point_empty = true;

		traj_.push_back(pt);
	}
}



