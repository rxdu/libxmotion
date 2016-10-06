/*
 * pos_step_response.cpp
 *
 *  Created on: Oct 5, 2016
 *      Author: rdu
 */

#include <cmath>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/comm.hpp"
#include "common/control_types.h"

using namespace srcl_ctrl;

srcl_msgs::UAVTrajectory_t GenerateTestTrajectory()
{
	srcl_msgs::UAVTrajectory_t test_traj;

	int time_stamp1 = 50;
	int time_stamp2 = time_stamp1 + 150;
	int final_time_stamp = time_stamp2 + 10;

	for(int i = 0; i < final_time_stamp; i++)
	{
		srcl_msgs::UAVTrajectoryPoint_t pt;
		pt.point_empty = false;

		double height = 0.5;
		double radius = 1.0;
		double circle_ang_vel = 180.0/180.0*3.14;

		if(i < time_stamp1)
		{
			pt.positions[0] = 0.0;
			pt.positions[1] = 0.0;
			pt.positions[2] = 0.5;
			pt.velocities[0] = 0;
			pt.velocities[1] = 0;
			pt.velocities[2] = 0;
			pt.accelerations[0] = 0;
			pt.accelerations[1] = 0;
			pt.accelerations[2] = 0;
			pt.yaw = 0;
			pt.duration = 1;
		}
		else if(i < time_stamp2)
		{
			pt.positions[0] = 1.0;
			pt.positions[1] = 1.0;
			pt.positions[2] = 0.5;
			pt.velocities[0] = 0;
			pt.velocities[1] = 0;
			pt.velocities[2] = 0;
			pt.accelerations[0] = 0;
			pt.accelerations[1] = 0;
			pt.accelerations[2] = 0;
			pt.yaw = 0;
			pt.duration = 1;
		}
		else
			pt.point_empty = true;

		test_traj.trajectory.push_back(pt);
	}

	test_traj.waypoint_num = test_traj.trajectory.size();

	return test_traj;
}

int main(int argc, char ** argv)
{
    lcm::LCM lcm;

    if(!lcm.good())
        return 1;

    srcl_msgs::UAVTrajectory_t traj = GenerateTestTrajectory();

    std::cout << "sending request" << std::endl;

    lcm.publish("quad_controller/quad_motion_service", &traj);

    std::cout << "finished sending request" << std::endl;

    return 0;
}


