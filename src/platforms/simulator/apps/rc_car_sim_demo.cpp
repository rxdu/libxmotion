/*
 * rc_car_sim_demo.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <cmath>

#include "rc_car_sim/rc_car_sim_client.hpp"
#include "vrep_interface/vrep_sim_process.hpp"

using namespace librav;

int main(int arc, char* argv[])
{
	std::shared_ptr<RCCarSimClient> client = std::make_shared<RCCarSimClient>();

	// create a simulation process
	VrepSimProcess<DataFromRCCarSim, DataToRCCarSim> process(client);

	// run the simulation in synchronous mode
	if(process.ConnectToServer())
		process.StartSimLoop_ASynchronous();

	return 0;
}


