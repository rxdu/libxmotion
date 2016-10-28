/*
 * quad_poly_traj_handler.h
 *
 *  Created on: Oct 28, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_QUAD_POLY_TRAJ_HANDLER_H_
#define CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_QUAD_POLY_TRAJ_HANDLER_H_

#include <memory>
#include <string>
#include <atomic>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

#include "common/control_types.h"
#include "quad_flat/quad_flattraj.h"

namespace srcl_ctrl {

class QuadPolyTrajHandler {
public:
	QuadPolyTrajHandler(std::shared_ptr<lcm::LCM> lcm);
	QuadPolyTrajHandler(std::shared_ptr<lcm::LCM> lcm, std::string planner_topic, std::string server_topic);
	~QuadPolyTrajHandler();

	friend class MotionServer;

private:
	std::shared_ptr<lcm::LCM> lcm_;
	std::string planner_topic_;
	std::string server_topic_;
	double step_size_;
	std::atomic<bool> traj_available_;
	std::atomic<time_stamp> current_sys_time_;
	time_stamp traj_start_time_;

	QuadFlatTraj flat_traj_;

	double GetRefactoredTime(double ts, double te, double t);
	void UpdateSystemTime(double t) { current_sys_time_ = t; };

public:
	void LcmPolyTrajMsgHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::PolynomialCurve_t* msg);
	UAVTrajectoryPoint GetDesiredTrajectoryPoint(time_t tstamp);
};

}

#endif /* CONTROL_SRC_QUAD_CTRL_MOTION_SERVER_QUAD_POLY_TRAJ_HANDLER_H_ */
