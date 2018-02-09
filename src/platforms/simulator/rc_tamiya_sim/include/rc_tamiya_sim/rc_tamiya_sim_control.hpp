/* 
 * rc_tamiya_sim_control.hpp
 * 
 * Created on: Aug 10, 2017
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef RC_TAMIYA_SIM_CONTROL_HPP
#define RC_TAMIYA_SIM_CONTROL_HPP

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "rc_tamiya_sim/rc_tamiya_sim_types.hpp"

namespace librav
{

class RCTamiyaSimControl
{
public:
	RCTamiyaSimControl();

	// DataToRCTamiyaSim ConvertRobotCmdToSimCmd(const RCCarCmd &cmd);
	void UpdateRobotState(const DataFromRCTamiyaSim &data);
	DataToRCTamiyaSim UpdateCtrlLoop();

	void SetInitPose(float x, float y, float yaw);
	void InitLogger(std::string log_name_prefix, std::string log_save_path);

private:
	std::shared_ptr<lcm::LCM> lcm_;
};
}

#endif /* RC_TAMIYA_SIM_CONTROL_HPP */
