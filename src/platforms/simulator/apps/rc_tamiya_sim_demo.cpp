/*
 * rc_car_sim_demo.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <cmath>

#include "rc_tamiya_sim/rc_tamiya_sim_client.hpp"
#include "vrep_interface/vrep_sim_process.hpp"

using namespace librav;

int main(int arc, char* argv[])
{
	std::shared_ptr<RCTamiyaSimClient> client = std::make_shared<RCTamiyaSimClient>();

	// create a simulation process
	VrepSimProcess<DataFromRCTamiyaSim, DataToRCTamiyaSim> process(client);

	// run the simulation in synchronous mode
	if(process.ConnectToServer())
		process.StartSimLoop_Synchronous();

	return 0;
}


