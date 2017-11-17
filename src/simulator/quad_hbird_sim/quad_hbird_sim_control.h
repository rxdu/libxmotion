/* 
 * quad_hbird_sim_control.h
 * 
 * Created on: Sep 2, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef QUAD_HBIRD_SIM_CONTROL_H
#define QUAD_HBIRD_SIM_CONTROL_H

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "vrep_sim/vrep_interface/robot_sim_control.h"
#include "simulator/quad_hbird_sim/quad_hbird_sim_types.h"

#include "control/quad_ctrl/state/quad_state.h"
#include "control/quad_ctrl/controller/att_quat_con.h"
#include "control/quad_ctrl/controller/pos_quat_con.h"
#include "control/quad_ctrl/actuator/quad_mixer.h"

#include "quadrotor/motion_server/motion_server.h"
#include "quadrotor/driver/quad_data_broadcaster.h"

namespace librav {

class QuadHbirdSimControl: public RobotSimControl<DataFromQuadSim, DataToQuadSim,QuadState, QuadCmd>
{
public:
	QuadHbirdSimControl();

private:
	UAVTrajectoryPoint previous_state_;

	std::unique_ptr<AttQuatCon> att_quat_con_;
	std::unique_ptr<PosQuatCon> pos_quat_con_;
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

#endif /* QUAD_HBIRD_SIM_CONTROL_H */
