/*
 * main.cpp
 *
 *  Created on: Jul 18, 2015
 *      Author: rdu
 */

// headers for standard library
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <math.h>
#include <ctime>
#include <pwd.h>

// headers for vrep remote api
extern "C" {
    #include "extApi.h"
//	#include "extApiCustom.h" // custom remote API functions
}

// headers for g3log
#ifdef ENABLE_LOG
#include "g3log/g3log.hpp"
#include "g3log/logworker.hpp"
#include "g3log/std2_make_unique.hpp"
#endif

// headers for user code
#include "sim_process/quad_sim_process.h"
#include "control/motion/trajectory_manager.h"
#include "main.h"

#ifdef ENABLE_LOG
using namespace g3;
#endif

using namespace srcl_ctrl;

#define USE_FIXED_PORT_NUM

#ifdef USE_FIXED_PORT_NUM
#define EXP_PORT_NUM 29999
#endif

uint64_t sim_time = 0;
TrajectoryPoint last_state;

int main(int argc,char* argv[])
{
	int portNb = 0;
	unsigned long loop_count = 0;

#ifdef USE_FIXED_PORT_NUM
	portNb = EXP_PORT_NUM;
#else
	if (argc>=1)
	{
		portNb=atoi(argv[1]);
	}
	else
	{
		printf("Please indicate following argument(s): 'portNumber'!\n");
		extApi_sleepMs(5000);
		return 0;
	}
#endif

#ifdef ENABLE_LOG
	// initialize logger
	auto worker = LogWorker::createLogWorker();
	// get home path of current user
	passwd* pw = getpwuid(getuid());
	std::string home_path(pw->pw_dir);
	auto defaultHandler = worker->addDefaultLogger("hummingbird_sim",
			home_path + "/Workspace/srcl_robot_suite/srcl_ctrl/control/hummingbird/quad_ctrl/data/log");
	initializeLogging(worker.get());

	/* add log data head */
			// from robot_state
	LOG(INFO) << "pos_real.x, " << "pos_real.y, " << "pos_real.z, "
			<< "vel_real.x, " << "vel_real.y, "<< "vel_real.z, "
			<< "euler_real.alpha, "<< "euler_real.beta, "<< "euler_real.gamma, "
			<< "quat_real.w, "<< "quat_real.x, "<< "quat_real.y, "<< "quat_real.z, "
			<< "rot_rate_real.x, "<< "rot_rate_real.y, "<< "rot_rate_real.z, "
			// from pos_quat_con
//
			// from att_quat_con
			<< "quat_d.w, "<< "quat_d.x, "<< "quat_d.y, "<< "quat_d.z, "
			<< "axis_dot_product.x, " << "axis_dot_product.y, " << "axis_dot_product.z, "
			<< "quat_e.w, "<< "quat_e.x, "<< "quat_e.y, "<< "quat_e.z, "
			<< "quat_e_wsign.x, "<< "quat_e_wsign.y, "<< "quat_e_wsign.z, "
			<< "rot_rate_e.x, "<< "rot_rate_e.y, "<< "rot_rate_e.z, "
			<< "desired_ft.0, "<< "desired_ft.1, "<< "desired_ft.2, "<< "desired_ft.3, "
			<< "motor_vel.0, "<< "motor_vel.1, "<< "motor_vel.2, " << "motor_vel.3";
#endif

	// initialize simulator
	simxInt clientID = simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);

	if (clientID!=-1)
	{
		std::cout << "INFO: Connected to server." << std::endl;

		// initialize a simulation process
		QuadSimProcess sim_process(clientID);
		simxInt ping_time = 0;
		TrajectoryManager traj_manager;
		last_state.point_empty = false;
		last_state.positions[0] = 0;
		last_state.positions[1] = 0;
		last_state.positions[2] = 0;

		std::cout << "INFO: Created a simulation client." << std::endl;

		simxSynchronous(clientID,true);
		simxStartSimulation(clientID, simx_opmode_oneshot_wait);

		std::cout << "INFO: Enabled synchronous mode." << std::endl;

		while (simxGetConnectionId(clientID)!=-1)
		{
			if(loop_count == 0)
				std::cout << "INFO: Entered control loop." << std::endl;

			// update simulated control loop
			if(sim_process.ReceiveDataFromSimulator())
			{
				// fetch the latest trajectory waypoint
				TrajectoryPoint pt;
				pt = traj_manager.GetTrajectoryPoint(sim_time);

				// if no new point, stay where it was
				if(!pt.point_empty)
				{
					sim_process.SimLoopUpdate(pt);
					last_state = pt;
				}
				else
					sim_process.SimLoopUpdate(last_state);
			}
//			else
//				std::cout<<"failed to fetch data from simulator"<<std::endl;

			// send command to robot
			sim_process.SendDataToSimulator();

			//extApi_sleepMs(1); 		// use usleep(1750) to get shorter delay
			usleep(50);

			// send trigger to simulator
			simxSynchronousTrigger(clientID);

			// every time a trigger is sent, simulation time forwards 10ms
			sim_time++;
			loop_count++;

			// after this call, the first simulation step is finished (Blocking function call)
			simxGetPingTime(clientID, &ping_time);
		}

		std::cout << "INFO: Exited control loop." << std::endl;

		simxStopSimulation(clientID, simx_opmode_oneshot_wait);

		simxFinish(clientID);
	}
	else
	{
		std::cout << "ERROR: Failed to connect to server" << std::endl;
	}

	std::cout << "INFO: Simulation ends." << std::endl;

	return(0);
}
