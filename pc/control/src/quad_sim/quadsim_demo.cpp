/*
 * quadsim_demo.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#include <iostream>

#include "quad_sim/quad_sim_client.h"
#include "quad_sim/quad_sim_controller.h"
#include "vrep_sim/vrep_interface/robot_sim_process.h"

using namespace srcl_ctrl;

int main(int arc, char* argv[])
{
	QuadSimClient* client = new QuadSimClient();
	QuadSimController* controller = new QuadSimController();

	RobotSimProcess<QuadDataFromSim, QuadDataToSim,QuadState, QuadCmd> process(client,controller);

	if(process.ConnectToServer())
		process.StartSimLoop_Synchronous();
}


