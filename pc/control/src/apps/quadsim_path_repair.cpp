/*
 * quadsim_path_repair.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <cmath>

#include "quad_sim/quad_sim_client.h"
#include "quad_sim/quad_sim_controller.h"
#include "vrep_sim/vrep_interface/robot_sim_process.h"

using namespace srcl_ctrl;

int main(int arc, char* argv[])
{
	std::shared_ptr<QuadSimClient> client = std::make_shared<QuadSimClient>();
	std::shared_ptr<QuadSimController> controller = std::make_shared<QuadSimController>();

	// set quadrotor init pose
//	controller->SetInitPose(-1.8,1,0.5,-M_PI/4);
	controller->SetInitPose(-1.8,0.6,1.0,0);
	controller->SendRobotStateToROS(true);

	// create a simulation process
	RobotSimProcess<QuadDataFromSim, QuadDataToSim,QuadState, QuadCmd> process(client,controller);

	// run the simulation in synchronous mode
	if(process.ConnectToServer())
		process.StartSimLoop_Synchronous();

	return 1;
}


