/*
 * quad_solo_sim_control.h
 *
 *  Created on: Oct 22, 2016
 *      Author: rdu
 */

#ifndef SIMULATOR_QUAD_SOLO_SIM_CONTROL_H_
#define SIMULATOR_QUAD_SOLO_SIM_CONTROL_H_

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>

#include "vrep_sim/vrep_interface/robot_sim_control.h"
#include "simulator/quad_solo_sim/quad_solo_sim_types.h"

#include "control/quad_ctrl/state/quad_state.h"
#include "control/quad_ctrl/controller/att_quat_con.h"
#include "control/quad_ctrl/controller/pos_quat_con.h"
#include "control/quad_ctrl/actuator/quad_mixer.h"

#include "platforms/quadrotor/motion_server/motion_server.h"
#include "platforms/quadrotor/driver/quad_data_broadcaster.h"

namespace librav {

class QuadSoloSimControl: public RobotSimControl<DataFromQuadSim, DataToQuadSim, QuadState, QuadCmd>
{
public:
	QuadSoloSimControl();
	UAVTrajectoryPoint previous_state_;

	std::unique_ptr<AttQuatCon> att_con_;
	std::unique_ptr<PosQuatCon> pos_con_;
	std::unique_ptr<QuadMixer>	mixer_;

	std::shared_ptr<lcm::LCM> lcm_;

	std::shared_ptr<MotionServer> motion_server_;

	bool broadcast_rs_;
	std::shared_ptr<QuadDataBroadcaster> data_trans_;

public:
	virtual DataToQuadSim ConvertRobotCmdToSimCmd(const QuadCmd& cmd);
	virtual void UpdateRobotState(const DataFromQuadSim& data);
	virtual QuadCmd UpdateCtrlLoop();

public:
	void SetInitPose(float x, float y, float z, float yaw);
	void BroadcastRobotState(bool cmd) { broadcast_rs_ = cmd; };
	void InitLogger(std::string log_name_prefix, std::string log_save_path);
	void SetMotionMode(MotionMode mode) { motion_server_->SetMotionMode(mode); };
};

}

#endif /* SIMULATOR_QUAD_SOLO_SIM_CONTROL_H_ */
