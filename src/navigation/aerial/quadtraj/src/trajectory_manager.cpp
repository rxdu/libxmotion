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

#include "quadtraj/trajectory_manager.hpp"

using namespace librav;

TrajectoryManager::TrajectoryManager(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm)
{
	lcm_->subscribe("quad_planner/goal_waypoints", &TrajectoryManager::LcmWaypointsHandler, this);
	lcm_->subscribe("quad_planner/goal_keyframe_set", &TrajectoryManager::LcmKeyframeSetHandler, this);
}

double TrajectoryManager::GetRefactoredTime(double ts, double te, double t)
{
	if (t < ts)
		t = ts;
	if (t > te)
		t = te;

	return (t - ts) / (te - ts);
}

double TrajectoryManager::CalcFlightTime(Position3Dd start, Position3Dd goal, double vel)
{
	double xe = start.x - goal.x;
	double ye = start.y - goal.y;
	double ze = start.z - goal.z;

	double dist = std::sqrt(std::pow(xe, 2) + std::pow(ye, 2) + std::pow(ze, 2));
	std::cout << "dist: " << dist << ", allocated time: " << dist / vel << std::endl;

	return dist / vel;
}

void TrajectoryManager::GenerateTrajectory(KeyframeSet &kfs, uint64_t traj_id)
{
	std::vector<Position3Dd> waypoints;

	uint8_t kf_num = kfs.keyframes.size();
	if (kf_num < 2)
		return;

	QuadPolyOpt traj_opt_;
	//	traj_opt_.InitOptJointMatrices(kf_num);
	traj_opt_.InitOptWithCorridorJointMatrices(kf_num, 20, 0.01);

	for (int i = 0; i < kfs.keyframes.size(); i++)
	{
		traj_opt_.keyframe_x_vals_(0, i) = kfs.keyframes[i].position[0];
		traj_opt_.keyframe_y_vals_(0, i) = kfs.keyframes[i].position[1];
		traj_opt_.keyframe_z_vals_(0, i) = kfs.keyframes[i].position[2];

		if (kfs.keyframes[i].vel_constr)
		{
			traj_opt_.keyframe_x_vals_(1, i) = kfs.keyframes[i].velocity[0];
			traj_opt_.keyframe_y_vals_(1, i) = kfs.keyframes[i].velocity[1];
			traj_opt_.keyframe_z_vals_(1, i) = kfs.keyframes[i].velocity[2];
		}
		else
		{
			traj_opt_.keyframe_x_vals_(1, i) = std::numeric_limits<float>::infinity();
			traj_opt_.keyframe_y_vals_(1, i) = std::numeric_limits<float>::infinity();
			traj_opt_.keyframe_z_vals_(1, i) = std::numeric_limits<float>::infinity();
		}

		traj_opt_.keyframe_x_vals_(2, i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(2, i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(2, i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_x_vals_(3, i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(3, i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(3, i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_yaw_vals_(0, i) = kfs.keyframes[i].yaw;
		traj_opt_.keyframe_yaw_vals_(1, i) = std::numeric_limits<float>::infinity();

		//traj_opt_.keyframe_ts_(0,i) = i * 1.0;

		if (i == 0)
			traj_opt_.keyframe_ts_(0, i) = 0;
		else
			traj_opt_.keyframe_ts_(0, i) = traj_opt_.keyframe_ts_(0, i - 1) + 1.0;

		waypoints.push_back(Position3Dd(kfs.keyframes[i].position[0], kfs.keyframes[i].position[1], kfs.keyframes[i].position[2]));
	}

	traj_opt_.keyframe_x_vals_(1, 0) = 0.0;
	traj_opt_.keyframe_y_vals_(1, 0) = 0.0;
	traj_opt_.keyframe_z_vals_(1, 0) = 0.0;

	traj_opt_.keyframe_x_vals_(2, 0) = 0.0;
	traj_opt_.keyframe_y_vals_(2, 0) = 0.0;
	traj_opt_.keyframe_z_vals_(2, 0) = 0.0;

	traj_opt_.keyframe_x_vals_(3, 0) = 0.0;
	traj_opt_.keyframe_y_vals_(3, 0) = 0.0;
	traj_opt_.keyframe_z_vals_(3, 0) = 0.0;

	traj_opt_.keyframe_yaw_vals_(1, 0) = 0;

	traj_opt_.keyframe_x_vals_(1, kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(1, kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(1, kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(2, kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(2, kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(2, kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(3, kf_num - 1) = 0;
	traj_opt_.keyframe_y_vals_(3, kf_num - 1) = 0;
	traj_opt_.keyframe_z_vals_(3, kf_num - 1) = 0;

	traj_opt_.keyframe_yaw_vals_(1, kf_num - 1) = 0;

	//traj_opt_.OptimizeFlatTrajJoint();
	bool result = traj_opt_.OptimizeFlatTrajWithCorridorJoint();

	// send results to LCM network
	if (result)
	{
		traj_available_ = false;
		active_trajectory_.clear();
		waypoints_.clear();

		// update new waypoints and trajectory
		waypoints_ = waypoints;
		active_trajectory_ = traj_opt_.flat_traj_;

		//	std::cout << "msg seg size: " << msg->segments.size() << std::endl;
		//	std::cout << "flat traj seg size: " << active_trajectory_.traj_segs_.size() << std::endl;
		//	std::cout << "way point size: " << waypoints_.size() << std::endl;

		traj_start_time_ = kfs.start_time;
		scaling_factor_ = 0.3;
		traj_id_ = traj_id;

		traj_available_ = true;

		SendActiveTrajectoryToLCM();
	}
}

UAVTrajectoryPoint TrajectoryManager::GetCurrentDesiredState(TimeStamp t)
{
	UAVTrajectoryPoint pt;

	pt.point_empty = true;

	if (traj_available_)
	{
		// get current time
		TimeStamp time = t - traj_start_time_;
		double t = time / 1000.0 / scaling_factor_;

		// check if traj is defined for the given time
		if (active_trajectory_.traj_segs_.size() == 0 || t > active_trajectory_.traj_segs_.back().t_end)
		{
			traj_available_ = false;
			return pt;
		}

		//std::cout << "request traj time: " << t << std::endl;

		// search for the right segment
		int seg_idx;
		for (seg_idx = 0; seg_idx < active_trajectory_.traj_segs_.size(); seg_idx++)
		{
			if (t >= active_trajectory_.traj_segs_[seg_idx].t_start && t <= active_trajectory_.traj_segs_[seg_idx].t_end)
				break;
		}

		if (seg_idx >= active_trajectory_.traj_segs_.size())
			return pt;

		std::cout << "active segment index: " << seg_idx << " of " << active_trajectory_.traj_segs_.size() << std::endl;
		next_wp_idx_ = seg_idx + 1;

		double seg_t_start = active_trajectory_.traj_segs_[seg_idx].t_start;
		double seg_t_end = active_trajectory_.traj_segs_[seg_idx].t_end;
		double t_factor = GetRefactoredTime(seg_t_start, seg_t_end, t);

		pt.point_empty = false;

		pt.positions[0] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_x.param_.coeffs, 0, t_factor);
		pt.positions[1] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_y.param_.coeffs, 0, t_factor);
		pt.positions[2] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_z.param_.coeffs, 0, t_factor);

		pt.velocities[0] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_x.param_.coeffs, 1, t_factor);
		pt.velocities[1] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_y.param_.coeffs, 1, t_factor);
		pt.velocities[2] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_z.param_.coeffs, 1, t_factor);

		pt.accelerations[0] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_x.param_.coeffs, 2, t_factor);
		pt.accelerations[1] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_y.param_.coeffs, 2, t_factor);
		pt.accelerations[2] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_z.param_.coeffs, 2, t_factor);

		pt.jerks[0] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_x.param_.coeffs, 3, t_factor);
		pt.jerks[1] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_y.param_.coeffs, 3, t_factor);
		pt.jerks[2] = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_z.param_.coeffs, 3, t_factor);

		pt.yaw = PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_yaw.param_.coeffs, 0, t_factor);
		pt.yaw_rate = 0; //PolynomialMath::GetPolynomialValue(active_trajectory_.traj_segs_[seg_idx].seg_yaw.param_.coeffs, 1, t_factor);

		// calculate remaining distance to goal
		double dist = 0;
		// if active segment is the last segment
		if (seg_idx == active_trajectory_.traj_segs_.size() - 1)
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
			//for(int i = seg_idx + 1; i < active_trajectory_.traj_segs_.size() - 1; i++)
			for (int i = seg_idx; i < active_trajectory_.traj_segs_.size() - 1; i++)
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

void TrajectoryManager::SendActiveTrajectoryToLCM()
{
	librav_lcm_msgs::PolynomialCurve_t poly_msg;

	// copy waypoints
	poly_msg.wp_num = waypoints_.size();
	for (auto &wp : waypoints_)
	{
		librav_lcm_msgs::WayPoint_t wpoint;
		wpoint.positions[0] = wp.x;
		wpoint.positions[1] = wp.y;
		wpoint.positions[2] = wp.z;
		poly_msg.waypoints.push_back(wpoint);
	}

	// copy trajectory segments
	poly_msg.seg_num = active_trajectory_.traj_segs_.size();
	for (auto &seg : active_trajectory_.traj_segs_)
	{
		librav_lcm_msgs::PolyCurveSegment_t seg_msg;

		seg_msg.coeffsize_x = seg.seg_x.param_.coeffs.size();
		seg_msg.coeffsize_y = seg.seg_y.param_.coeffs.size();
		seg_msg.coeffsize_z = seg.seg_z.param_.coeffs.size();
		seg_msg.coeffsize_yaw = seg.seg_yaw.param_.coeffs.size();
		for (auto &coeff : seg.seg_x.param_.coeffs)
			seg_msg.coeffs_x.push_back(coeff);
		for (auto &coeff : seg.seg_y.param_.coeffs)
			seg_msg.coeffs_y.push_back(coeff);
		for (auto &coeff : seg.seg_z.param_.coeffs)
			seg_msg.coeffs_z.push_back(coeff);
		for (auto &coeff : seg.seg_yaw.param_.coeffs)
			seg_msg.coeffs_yaw.push_back(coeff);

		seg_msg.t_start = seg.t_start;
		seg_msg.t_end = seg.t_end;

		poly_msg.segments.push_back(seg_msg);
	}

	// copy time coefficients
	poly_msg.start_time.time_stamp = traj_start_time_;
	poly_msg.trajectory_id = traj_id_;
	poly_msg.scaling_factor = scaling_factor_;

	// send trajectory to LCM network
	lcm_->publish("quad_planner/trajectory_polynomial", &poly_msg);
}

void TrajectoryManager::LcmWaypointsHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::Path_t *msg)
{
	std::cout << "waypoints received: " << msg->waypoint_num << std::endl;

	// generate keyframes from waypoint
	KeyframeSet new_kfs;
	for (int i = 0; i < msg->waypoint_num; i++)
	{
		Keyframe kf;

		for (int j = 0; j < 3; j++)
			kf.position[j] = msg->waypoints[i].positions[j];

		kf.vel_constr = false;
		kf.yaw = msg->waypoints[i].yaw;
		new_kfs.keyframes.push_back(kf);
	}

	GenerateTrajectory(new_kfs, user_path_id_++);
}

void TrajectoryManager::LcmKeyframeSetHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::KeyframeSet_t *msg)
{
	std::cout << "keyframes received: " << msg->kf_num << std::endl;

	// copy keyframes from msg
	KeyframeSet new_kfs;
	for (int i = 0; i < msg->kf_num; i++)
	{
		Keyframe kf;

		for (int j = 0; j < 3; j++)
		{
			kf.position[j] = msg->kfs[i].position[j];
			kf.velocity[j] = msg->kfs[i].velocity[j];
		}
		kf.vel_constr = msg->kfs[i].vel_constr;
		kf.yaw = msg->kfs[i].yaw;

		new_kfs.keyframes.push_back(kf);
	}
	new_kfs.start_time = msg->sys_time.time_stamp;

	GenerateTrajectory(new_kfs, msg->path_id);
}

void TrajectoryManager::ReportProgress(void)
{
	if (traj_available_)
	{
		librav_lcm_msgs::MissionInfo_t info_msg;

		info_msg.trajectory_id = traj_id_;
		info_msg.dist_to_goal = remaining_dist_;
		info_msg.next_wp_id = next_wp_idx_;

		lcm_->publish("quad_ctrl/mission_info", &info_msg);
	}
}