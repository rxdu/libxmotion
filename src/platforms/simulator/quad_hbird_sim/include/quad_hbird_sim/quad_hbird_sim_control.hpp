/* 
 * quad_hbird_sim_control.hpp
 * 
 * Created on: Sep 2, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef QUAD_HBIRD_SIM_CONTROL_HPP
#define QUAD_HBIRD_SIM_CONTROL_HPP

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "quad_hbird_sim/quad_hbird_sim_client.hpp"
#include "quad_hbird_sim/quad_hbird_sim_types.hpp"

#include "control/quad_state.hpp"
#include "control/att_quat_con.hpp"
#include "control/pos_quat_con.hpp"
#include "control/quad_mixer.hpp"

#include "platforms/quadrotor/motion_server/motion_server.h"
#include "platforms/quadrotor/driver/quad_data_broadcaster.h"

namespace librav {

class QuadHbirdSimControl
{
public:
	QuadHbirdSimControl();
	~QuadHbirdSimControl() = default;

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
	void UpdateRobotState(const DataFromQuadSim& data);
	DataToQuadSim UpdateCtrlLoop();

public:
	void SetInitPose(float x, float y, float z, float yaw);
	void BroadcastRobotState(bool cmd) { broadcast_rs_ = cmd; };
	void InitLogger(std::string log_name_prefix, std::string log_save_path);
	void SetMotionMode(MotionMode mode) { motion_server_->SetMotionMode(mode); };
};

}

#endif /* QUAD_HBIRD_SIM_CONTROL_HPP */
