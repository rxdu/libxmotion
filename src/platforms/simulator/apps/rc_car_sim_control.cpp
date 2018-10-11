/*
 * rc_car_sim_demo.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <cmath>

#include "rc_car_sim/rc_car_sim_client.h"
#include "rc_car_sim/rc_car_sim_control.h"
#include "vrep_sim/vrep_interface/robot_sim_process.h"

using namespace librav;

int main(int arc, char* argv[])
{
	std::shared_ptr<RCTamiyaSimClient> client = std::make_shared<RCTamiyaSimClient>();
	std::shared_ptr<RCTamiyaSimControl> control = std::make_shared<RCTamiyaSimControl>();

	// create a simulation process
	RobotSimProcess<DataFromRCTamiyaSim, DataToRCTamiyaSim,RCCarState, RCCarCmd> process(client,control);

	// run the simulation in synchronous mode
	if(process.ConnectToServer())
		process.StartSimLoop_ASynchronous();

	return 0;
}


