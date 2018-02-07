/*
 * quadsim_hbird_demo.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <cmath>
#include <string>
#include <stdlib.h>

#include "vrep_interface/vrep_sim_process.hpp"

#include "quad_hbird_sim/quad_hbird_sim_client.hpp"

using namespace librav;

int main(int arc, char* argv[])
{
	std::shared_ptr<QuadHbirdSimClient> client = std::make_shared<QuadHbirdSimClient>();

	// set quadrotor init pose
	//controller->SetInitPose(-1.8,2,0.5,-M_PI/4);
	client->quad_ctrl.SetInitPose(0,0,0.5,0);
	client->quad_ctrl.BroadcastRobotState(true);
	client->quad_ctrl.InitLogger("quadsim_hummingbird", "/quad/control");

	// create a simulation process
	VrepSimProcess<DataFromQuadSim, DataToQuadSim> process(client);

	// run the simulation in synchronous mode
	if(process.ConnectToServer())
		process.StartSimLoop_Synchronous();

	return 1;
}


