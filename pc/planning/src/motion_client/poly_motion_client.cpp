/*
 * poly_motion_client.cpp
 *
 *  Created on: Sep 26, 2016
 *      Author: rdu
 */

#include <iostream>

#include "motion_client/poly_motion_client.h"
#include "polyopt/polyopt_math.h"

using namespace srcl_ctrl;

PolyMotionClient::PolyMotionClient(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		planner_topic_("quad_planner/trajectory_polynomial"),
		server_topic_("quad_controller/quad_motion_service"),
		step_size_(0.01)
{
	if(!lcm_->good())
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
	else {
		lcm_->subscribe(planner_topic_,&PolyMotionClient::LcmPolynomialHandler, this);
	}
}

PolyMotionClient::PolyMotionClient(std::shared_ptr<lcm::LCM> lcm, std::string planner_topic, std::string server_topic):
		lcm_(lcm),
		planner_topic_(planner_topic),
		server_topic_(server_topic),
		step_size_(0.01)
{
	if(!lcm_->good())
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
	else {
		lcm_->subscribe(planner_topic_,&PolyMotionClient::LcmPolynomialHandler, this);
	}
}

PolyMotionClient::~PolyMotionClient()
{

}

double PolyMotionClient::GetRefactoredTime(double ts, double te, double t)
{
	if(t < ts)
		t = ts;
	if(t > te)
		t = te;

	return (t - ts) / (te - ts);
}

void PolyMotionClient::LcmPolynomialHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::PolynomialCurve_t* msg)
{
	srcl_msgs::UAVTrajectory_t traj_msg;

	std::cout << "polynomial msg received in motion client" << std::endl;

	for(auto& seg:msg->segments)
	{
		double t;

		for(t = seg.t_start; t < seg.t_end; t += step_size_)
		{
			double t_factor = GetRefactoredTime(seg.t_start, seg.t_end, t);
			srcl_msgs::UAVTrajectoryPoint_t pt;

			pt.point_empty = false;

			pt.positions[0] = PolyOptMath::GetPolynomialValue(seg.coeffs_x, 0, t_factor);
			pt.positions[1] = PolyOptMath::GetPolynomialValue(seg.coeffs_y, 0, t_factor);
			pt.positions[2] = PolyOptMath::GetPolynomialValue(seg.coeffs_z, 0, t_factor);

			pt.velocities[0] = PolyOptMath::GetPolynomialValue(seg.coeffs_x, 1, t_factor);
			pt.velocities[1] = PolyOptMath::GetPolynomialValue(seg.coeffs_y, 1, t_factor);
			pt.velocities[2] = PolyOptMath::GetPolynomialValue(seg.coeffs_z, 1, t_factor);

			pt.accelerations[0] = PolyOptMath::GetPolynomialValue(seg.coeffs_x, 2, t_factor);
			pt.accelerations[1] = PolyOptMath::GetPolynomialValue(seg.coeffs_y, 2, t_factor);
			pt.accelerations[2] = PolyOptMath::GetPolynomialValue(seg.coeffs_z, 2, t_factor);

			pt.yaw = -M_PI/4;//PolyOptMath::GetPolynomialValue(seg.coeffs_yaw, 0, t);
			pt.duration = step_size_;

			traj_msg.trajectory.push_back(pt);
		}
	}

	traj_msg.waypoint_num = traj_msg.trajectory.size();

	lcm_->publish(server_topic_, &traj_msg);
}

