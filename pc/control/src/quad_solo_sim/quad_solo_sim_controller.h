/*
 * quad_solo_sim_controller.h
 *
 *  Created on: Oct 22, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_SOLO_SIM_QUAD_SIM_CONTROLLER_H_
#define CONTROL_SRC_QUAD_SOLO_SIM_QUAD_SIM_CONTROLLER_H_

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

#include "ctrl_utils/logging/logging_helper.h"

namespace srcl_ctrl {

class QuadSoloSimController : public RobotSimController<QuadDataFromSim, QuadDataToSim,QuadState, QuadCmd>
{
public:
	QuadSoloSimController();
	~QuadSoloSimController();

private:
	UAVTrajectoryPoint previous_state_;

	AttQuatCon* att_quat_con_;
	PosQuatCon* pos_quat_con_;

	std::shared_ptr<lcm::LCM> lcm_;

	MotionServer motion_server_;

	bool broadcast_rs_;
	std::shared_ptr<QuadDataTransmitter> data_trans_;
	std::shared_ptr<LoggingHelper> logging_helper_;

public:
	virtual QuadDataToSim ConvertRobotCmdToSimCmd(const QuadCmd& cmd);

	virtual void UpdateRobotState(const QuadDataFromSim& data);

	virtual QuadCmd UpdateCtrlLoop();
	virtual QuadCmd UpdateCtrlLoop(const QuadState& desired);

public:
	void SetInitPose(float x, float y, float z, float yaw);
	void BroadcastRobotState(bool cmd) { broadcast_rs_ = cmd; };
};

}

#endif /* CONTROL_SRC_QUAD_SOLO_SIM_QUAD_SIM_CONTROLLER_H_ */
