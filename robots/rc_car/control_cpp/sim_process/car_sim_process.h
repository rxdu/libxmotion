/*
 * car_sim_process.h
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#ifndef SIM_PROCESS_CAR_SIM_PROCESS_H_
#define SIM_PROCESS_CAR_SIM_PROCESS_H_

#include "sim_process/sim_process.h"
#include <vrep_client/robot_sim_client.h>
#include "vrep_client/rc_car_client.h"
#include "perception/line_detector.h"

namespace RobotToolkitRIVeR
{

class CarSimProcess : public SimProcess
{
public:
	CarSimProcess(int client_id);
	~CarSimProcess();

public:
	void SimLoopUpdate();

private:
	unsigned long process_loop_count;
	LineDetector line_det_;

};

}


#endif /* SIM_PROCESS_CAR_SIM_PROCESS_H_ */
