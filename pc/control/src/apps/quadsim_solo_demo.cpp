/*
 * quadsim_demo.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <cmath>

#include "quad_solo_sim/quad_solo_sim_client.h"
#include "quad_solo_sim/quad_solo_sim_control.h"
#include "vrep_sim/vrep_interface/robot_sim_process.h"

using namespace srcl_ctrl;

int main(int arc, char* argv[])
{
	std::shared_ptr<QuadSoloSimClient> client = std::make_shared<QuadSoloSimClient>();
	std::shared_ptr<QuadSoloSimControl> control = std::make_shared<QuadSoloSimControl>();
	//controller->SetInitPose(-1.8,2,0.5,-M_PI/4);
	control->SetInitPose(0,0,0.5,0);
	control->BroadcastRobotState(true);
	char* home_path;
	home_path = getenv ("HOME");
	std::string log_path;
	if (home_path!=NULL)
	{
		std::string hm(home_path);
		log_path = hm+"/Workspace/srcl_rtk/srcl_ctrl/pc/control/log/quad";
	}
	else
	{
		// default path
		log_path = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/control/log/quad";
	}
	control->InitLogger("quadsim_solo.log", log_path);

	// create a simulation process
	RobotSimProcess<DataFromQuadSim, DataToQuadSim,QuadState, QuadCmd> process(client,control);

	// run the simulation in synchronous mode
	if(process.ConnectToServer())
		process.StartSimLoop_Synchronous();

	return 1;
}


