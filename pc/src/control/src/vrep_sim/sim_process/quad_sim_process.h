/*
 * car_sim_process.h
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#ifndef SIM_PROCESS_QUAD_SIM_PROCESS_H_
#define SIM_PROCESS_QUAD_SIM_PROCESS_H_

#include "vrep_sim/vrep_client/quad_sim_client.h"
#include "vrep_sim/vrep_client/robot_sim_client.h"

#include "vrep_sim/sim_process/sim_process.h"

#include "quad_ctrl/estimator/robot_state.h"
#include "quad_ctrl/controller/att_quat_con.h"
#include "quad_ctrl/controller/pos_quat_con.h"

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

	AttQuatCon* att_quat_con_;
	PosQuatCon* pos_quat_con_;

public:
	void SimLoopUpdate();
	void SimLoopUpdate(UAVTrajectoryPoint pt);
};

}


#endif /* SIM_PROCESS_QUAD_SIM_PROCESS_H_ */
