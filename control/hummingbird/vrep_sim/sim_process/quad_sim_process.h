/*
 * car_sim_process.h
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#ifndef SIM_PROCESS_QUAD_SIM_PROCESS_H_
#define SIM_PROCESS_QUAD_SIM_PROCESS_H_

#include <vrep_client/quad_sim_client.h>
#include "sim_process/sim_process.h"
#include <vrep_client/robot_sim_client.h>
#include "navigation/robot_state.h"
#include "control/pos_quat_con.h"

namespace srcl_ctrl
{

class QuadSimProcess : public SimProcess
{
public:
	QuadSimProcess(int client_id);
	~QuadSimProcess();

private:
	unsigned long process_loop_count;

	RobotState rs_;
	RobotState est_rs_;

	PosQuatCon* pos_quat_con;

public:
	void SimLoopUpdate();
};

}


#endif /* SIM_PROCESS_QUAD_SIM_PROCESS_H_ */
