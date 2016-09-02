/*
 * quadsim_demo.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#include <iostream>

#include "quad_sim/quad_sim_client.h"
#include "vrep_sim/vrep_interface/robot_sim_process.h"

using namespace srcl_ctrl;

int main(int arc, char* argv[])
{
	QuadSimClient* client = new QuadSimClient();
//	RobotSimProcess process(client);
//
//	if(process.ConnectToServer())
//		process.StartSimLoop_Synchronous();
}


