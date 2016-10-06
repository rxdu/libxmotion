/*
 * quad_sim_controller.h
 *
 *  Created on: Sep 2, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_SIM_QUAD_SIM_CONTROLLER_H_
#define CONTROL_SRC_QUAD_SIM_QUAD_SIM_CONTROLLER_H_

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>

#include "vrep_sim/vrep_interface/robot_sim_controller.h"
#include "quad_sim/quad_sim_data.h"

#include "quad_ctrl/estimator/quad_state.h"
#include "quad_ctrl/controller/quad_types.h"
#include "quad_ctrl/controller/att_quat_con.h"
#include "quad_ctrl/controller/pos_quat_con.h"
#include "quad_ctrl/motion_server/motion_server.h"
#include "quad_ctrl/data_trans/quad_data_transmitter.h"

namespace srcl_ctrl {

class QuadSimController : public RobotSimController<QuadDataFromSim, QuadDataToSim,QuadState, QuadCmd>
{
public:
	QuadSimController();
	~QuadSimController();

private:
	UAVTrajectoryPoint previous_state_;

	AttQuatCon* att_quat_con_;
	PosQuatCon* pos_quat_con_;

	std::shared_ptr<lcm::LCM> lcm_;

	MotionServer motion_server_;

	bool send_to_ros_;
	std::shared_ptr<QuadDataTransmitter> data_trans_;

public:
	virtual QuadDataToSim ConvertRobotCmdToSimCmd(const QuadCmd& cmd);

	virtual void UpdateRobotState(const QuadDataFromSim& data);

	virtual QuadCmd UpdateCtrlLoop();
	virtual QuadCmd UpdateCtrlLoop(const QuadState& desired);

public:
	void SetInitPose(float x, float y, float z, float yaw);
	void BroadcastRobotState(bool cmd) { send_to_ros_ = cmd; };
};

}

#endif /* CONTROL_SRC_QUAD_SIM_QUAD_SIM_CONTROLLER_H_ */
