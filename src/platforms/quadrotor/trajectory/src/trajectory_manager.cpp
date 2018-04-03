/* 
 * trajectory_manager.cpp
 * 
 * Created on: Apr 03, 2018 00:00
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <cmath>
#include <iostream>

#include "trajectory/trajectory_manager.hpp"

using namespace librav;

TrajectoryManager::TrajectoryManager(std::shared_ptr<lcm::LCM> lcm, std::string poly_traj_topic) : lcm_(lcm)
{
	lcm_->subscribe(poly_traj_topic, &TrajectoryManager::LcmPolyTrajMsgHandler, this);
}

void TrajectoryManager::ReportProgress(void)
{
	if (traj_available_)
	{
		srcl_lcm_msgs::MissionInfo_t info_msg;

		info_msg.trajectory_id = traj_id_;
		info_msg.dist_to_goal = remaining_dist_;
		info_msg.next_wp_id = next_wp_idx_;

		lcm_->publish("quad_ctrl/mission_info", &info_msg);
	}
}

double TrajectoryManager::GetRefactoredTime(double ts, double te, double t)
{
	if (t < ts)
		t = ts;
	if (t > te)
		t = te;

	return (t - ts) / (te - ts);
}

void TrajectoryManager::LcmPolyTrajMsgHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const srcl_lcm_msgs::PolynomialCurve_t *msg)
{
	//std::cout << "polynomial msg received" << std::endl;

	traj_available_ = false;
	flat_traj_.clear();
	waypoints_.clear();

	// copy data from msg
	for (auto &seg : msg->segments)
	{
		std::vector<std::vector<double>> segcoeffs;

		segcoeffs.push_back(seg.coeffs_x);
		segcoeffs.push_back(seg.coeffs_y);
		segcoeffs.push_back(seg.coeffs_z);
		segcoeffs.push_back(seg.coeffs_yaw);

		//std::cout << "start time: " << seg.t_start << " , end time: " << seg.t_end << std::endl;

		flat_traj_.AddTrajSeg(segcoeffs, seg.t_start, seg.t_end);
	}

	for (auto &wp : msg->waypoints)
		waypoints_.push_back(Position3Dd(wp.positions[0], wp.positions[1], wp.positions[2]));

	//	std::cout << "msg seg size: " << msg->segments.size() << std::endl;
	//	std::cout << "flat traj seg size: " << flat_traj_.traj_segs_.size() << std::endl;
	//	std::cout << "way point size: " << waypoints_.size() << std::endl;

	traj_start_time_ = msg->start_time.time_stamp;
	scaling_factor_ = msg->scaling_factor;
	traj_id_ = msg->trajectory_id;

	traj_available_ = true;
}

UAVTrajectoryPoint TrajectoryManager::GetCurrentDesiredState(time_stamp t)
{
	UAVTrajectoryPoint pt;

	pt.point_empty = true;

	if (traj_available_)
	{
		// get current time
		time_stamp time = t - traj_start_time_;
		double t = time / 1000.0 / scaling_factor_;

		// check if traj is defined for the given time
		if (flat_traj_.traj_segs_.size() == 0 || t > flat_traj_.traj_segs_.back().t_end)
		{
			traj_available_ = false;
			return pt;
		}

		//std::cout << "request traj time: " << t << std::endl;

		// search for the right segment
		int seg_idx;
		for (seg_idx = 0; seg_idx < flat_traj_.traj_segs_.size(); seg_idx++)
		{
			if (t >= flat_traj_.traj_segs_[seg_idx].t_start && t <= flat_traj_.traj_segs_[seg_idx].t_end)
				break;
		}

		if (seg_idx >= flat_traj_.traj_segs_.size())
			return pt;

		std::cout << "active segment index: " << seg_idx << " of " << flat_traj_.traj_segs_.size() << std::endl;
		next_wp_idx_ = seg_idx + 1;

		double seg_t_start = flat_traj_.traj_segs_[seg_idx].t_start;
		double seg_t_end = flat_traj_.traj_segs_[seg_idx].t_end;
		double t_factor = GetRefactoredTime(seg_t_start, seg_t_end, t);

		pt.point_empty = false;

		pt.positions[0] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 0, t_factor);
		pt.positions[1] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 0, t_factor);
		pt.positions[2] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 0, t_factor);

		pt.velocities[0] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 1, t_factor);
		pt.velocities[1] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 1, t_factor);
		pt.velocities[2] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 1, t_factor);

		pt.accelerations[0] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 2, t_factor);
		pt.accelerations[1] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 2, t_factor);
		pt.accelerations[2] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 2, t_factor);

		pt.jerks[0] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_x.param_.coeffs, 3, t_factor);
		pt.jerks[1] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_y.param_.coeffs, 3, t_factor);
		pt.jerks[2] = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_z.param_.coeffs, 3, t_factor);

		pt.yaw = PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_yaw.param_.coeffs, 0, t_factor);
		pt.yaw_rate = 0; //PolynomialMath::GetPolynomialValue(flat_traj_.traj_segs_[seg_idx].seg_yaw.param_.coeffs, 1, t_factor);

		// calculate remaining distance to goal
		double dist = 0;
		// if active segment is the last segment
		if (seg_idx == flat_traj_.traj_segs_.size() - 1)
		{
			dist = std::sqrt(std::pow(pt.positions[0] - waypoints_.back().x, 2) +
							 std::pow(pt.positions[1] - waypoints_.back().y, 2) + std::pow(pt.positions[2] - waypoints_.back().z, 2));
		}
		else
		{
			// calc remaining distance of the current segment
			//dist += std::sqrt(std::pow(pt.positions[0] - waypoints_[seg_idx + 1].x,2) +
			//		std::pow(pt.positions[1] - waypoints_[seg_idx + 1].y,2) + std::pow(pt.positions[2] - waypoints_[seg_idx + 1].z,2));

			// calc other waypoints
			//for(int i = seg_idx + 1; i < flat_traj_.traj_segs_.size() - 1; i++)
			for (int i = seg_idx; i < flat_traj_.traj_segs_.size() - 1; i++)
			{
				dist += std::sqrt(std::pow(waypoints_[i].x - waypoints_[i + 1].x, 2) +
								  std::pow(waypoints_[i].y - waypoints_[i + 1].y, 2));
				//+ std::pow(waypoints_[i].z - waypoints_[i + 1].z,2));
			}
		}

		if (dist < 0.01)
			dist = 0;

		remaining_dist_ = dist;

		//		std::cout << "estimated distance to goal: " << dist << std::endl;
	}

	return pt;
}

