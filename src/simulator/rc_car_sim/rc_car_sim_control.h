/*
 * rc_car_sim_control.h
 *
 *  Created on: Aug 10, 2017
 *      Author: rdu
 */

#ifndef SIMULATOR_RC_CAR_SIM_CONTROL_H_
#define SIMULATOR_RC_CAR_SIM_CONTROL_H_

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "vrep_sim/vrep_interface/robot_sim_control.h"
#include "simulator/rc_car_sim/rc_car_sim_types.h"
//#include "control/rc_car_ctrl/rc_car_state.h"

namespace librav
{

class RCCarSimControl : public RobotSimControl<DataFromRCCarSim, DataToRCCarSim, RCCarState, RCCarCmd>
{
  public:
	RCCarSimControl();

  private:
	std::shared_ptr<lcm::LCM> lcm_;

  public:
	virtual DataToRCCarSim ConvertRobotCmdToSimCmd(const RCCarCmd &cmd);
	virtual void UpdateRobotState(const DataFromRCCarSim &data);
	virtual RCCarCmd UpdateCtrlLoop();

  public:
	void SetInitPose(float x, float y, float yaw);
	void InitLogger(std::string log_name_prefix, std::string log_save_path);
};
}

#endif /* SIMULATOR_RC_CAR_SIM_CONTROL_H_ */
