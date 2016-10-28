/*
 * quadsim_path_repair.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <cmath>

#include "vrep_sim/vrep_interface/robot_sim_process.h"

#include "quad_hbird_sim/quad_hbird_sim_client.h"
#include "quad_hbird_sim/quad_hbird_sim_controller.h"

using namespace srcl_ctrl;

int main(int arc, char* argv[])
{
	std::shared_ptr<QuadHbirdSimClient> client = std::make_shared<QuadHbirdSimClient>();
	std::shared_ptr<QuadHbirdSimController> controller = std::make_shared<QuadHbirdSimController>();

	// set quadrotor init pose
//	controller->SetInitPose(-1.8,1,0.5,-M_PI/4);
//	controller->SetInitPose(-1.8,1.2,0.6,-M_PI/4); // demo for dynamic replanning from origin to fixed point
//	controller->SetInitPose(-1.8,0.6,0.8,-M_PI/4);
	controller->SetInitPose(-1.8,2,0.6,-M_PI/4);
	controller->BroadcastRobotState(true);
	controller->InitLogger("quadsim_hummingbird", "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/control/log/quad");

	// create a simulation process
	RobotSimProcess<DataFromQuadSim, DataToQuadSim,QuadState, QuadCmd> process(client,controller);

	// run the simulation in synchronous mode
	if(process.ConnectToServer())
		process.StartSimLoop_Synchronous();

	return 1;
}


