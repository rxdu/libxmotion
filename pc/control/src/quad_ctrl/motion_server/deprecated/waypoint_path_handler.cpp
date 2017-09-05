/*
 * waypoint_path_handler.cpp
 *
 *  Created on: Nov 1, 2016
 *      Author: rdu
 */

#include "motion_server/waypoint_path_handler.h"

using namespace srcl_ctrl;

WaypointPathHandler::WaypointPathHandler(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		path_topic_("quad_planner/path_waypoints"),
		traj_available_(false),
		current_sys_time_(0),
		traj_start_time_(0)
{

}

WaypointPathHandler::WaypointPathHandler(std::shared_ptr<lcm::LCM> lcm, std::string path_topic):
		lcm_(lcm),
		path_topic_(path_topic),
		traj_available_(false),
		current_sys_time_(0),
		traj_start_time_(0)
{

}

WaypointPathHandler::~WaypointPathHandler()
{

}

void WaypointPathHandler::LcmPolyTrajMsgHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::Path_t* msg)
{
	traj_available_ = false;



	traj_start_time_ = current_sys_time_;
	traj_available_ = true;
}

UAVTrajectoryPoint WaypointPathHandler::GetDesiredTrajectoryPoint(time_t tstamp)
{
	UAVTrajectoryPoint pt;

	pt.point_empty = true;

	if(traj_available_)
	{
		// get current time
		time_stamp time = tstamp - traj_start_time_;
		double t = time / 1000.0;

//		// check if traj is defined for the given time
//		if(flat_traj_.traj_segs_.size() == 0 || t > flat_traj_.traj_segs_.back().t_end)
//		{
//			traj_available_ = false;
//			return pt;
//		}
//
//		std::cout << "request traj time: " << t << std::endl;
//
//		// search for the right segment
//		int seg_idx;
//		for(seg_idx = 0; seg_idx < flat_traj_.traj_segs_.size(); seg_idx++)
//		{
//			if(t >= flat_traj_.traj_segs_[seg_idx].t_start && t <= flat_traj_.traj_segs_[seg_idx].t_end)
//				break;
//		}
//
//		if(seg_idx >= flat_traj_.traj_segs_.size())
//			return pt;
//
//		std::cout << "active segment index: " << seg_idx << std::endl;
//
//		double seg_t_start = flat_traj_.traj_segs_[seg_idx].t_start;
//		double seg_t_end = flat_traj_.traj_segs_[seg_idx].t_end;
//		double t_factor = GetRefactoredTime(seg_t_start, seg_t_end, t);
//
//		pt.point_empty = false;
//
//		pt.positions[0] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 0, t_factor);
//		pt.positions[1] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 0, t_factor);
//		pt.positions[2] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 0, t_factor);
//
//		pt.velocities[0] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 1, t_factor);
//		pt.velocities[1] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 1, t_factor);
//		pt.velocities[2] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 1, t_factor);
//
//		pt.accelerations[0] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 2, t_factor);
//		pt.accelerations[1] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 2, t_factor);
//		pt.accelerations[2] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 2, t_factor);
//
//		pt.jerks[0] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 3, t_factor);
//		pt.jerks[1] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 3, t_factor);
//		pt.jerks[2] = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 3, t_factor);
//
//		pt.yaw = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_yaw.param_.coeffs, 0, t_factor);
//		pt.yaw_rate = PolyOptMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_yaw.param_.coeffs, 1, t_factor);
	}

	return pt;
}

