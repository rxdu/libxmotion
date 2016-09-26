/*
 * poly_motion_client.h
 *
 *  Created on: Sep 26, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_MOTION_CLIENT_POLY_MOTION_CLIENT_H_
#define PLANNING_SRC_MOTION_CLIENT_POLY_MOTION_CLIENT_H_

#include <memory>
#include <string>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

namespace srcl_ctrl {

class PolyMotionClient
{
public:
	PolyMotionClient(std::shared_ptr<lcm::LCM> lcm);
	PolyMotionClient(std::shared_ptr<lcm::LCM> lcm, std::string planner_topic, std::string server_topic);
	~PolyMotionClient();

private:
	std::shared_ptr<lcm::LCM> lcm_;
	std::string planner_topic_;
	std::string server_topic_;
	double step_size_;

public:
	void LcmPolynomialHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::PolynomialCurve_t* msg);
};

}

#endif /* PLANNING_SRC_MOTION_CLIENT_POLY_MOTION_CLIENT_H_ */
