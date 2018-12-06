/*
 * rc_car_sim_demo.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#include <iostream>
#include <memory>
#include <cmath>

#include "datalink/lcm_link.hpp"

#include "rc_tamiya_sim/rc_tamiya_sim_client.hpp"
#include "vrep_interface/vrep_sim_process.hpp"

using namespace librav;

int main(int arc, char* argv[])
{
    std::shared_ptr<LCMLink> lcm = std::make_shared<LCMLink>();

    if (!lcm->good())
    {
        std::cout << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

	std::shared_ptr<RCTamiyaSimClient> client = std::make_shared<RCTamiyaSimClient>(lcm);

	// create a simulation process
	VrepSimProcess<DataFromRCTamiyaSim, DataToRCTamiyaSim> process(client);

	// run the simulation in synchronous mode
	if(process.ConnectToServer())
		process.StartSimLoop_ASynchronous();

	return 0;
}


