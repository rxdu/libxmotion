/*
 * trajectory_generator.cpp
 *
 *  Created on: Oct 31, 2016
 *      Author: rdu
 */

#include <mission/trajectory_generator.h>
#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"


using namespace srcl_ctrl;

TrajectoryGenerator::TrajectoryGenerator(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm)
{
	lcm_->subscribe("quad_planner/goal_waypoints",&TrajectoryGenerator::LcmWaypointsHandler, this);
	lcm_->subscribe("quad_planner/goal_keyframe_set",&TrajectoryGenerator::LcmKeyframeSetHandler, this);
}

TrajectoryGenerator::~TrajectoryGenerator()
{

}

void TrajectoryGenerator::LcmWaypointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::Path_t* msg)
{
	std::cout << "waypoints received: " << msg->waypoint_num << std::endl;

	// generate keyframes from waypoint
	KeyframeSet new_kfs;
	for(int i = 0; i < msg->waypoint_num; i++)
	{
		Keyframe kf;

		for(int j = 0; j < 3; j++)
			kf.positions[j] = msg->waypoints[i].positions[j];

		kf.vel_constr = false;
		kf.yaw = msg->waypoints[i].yaw;
		new_kfs.keyframes.push_back(kf);
	}

	GenerateTrajectory(new_kfs);
}

void TrajectoryGenerator::LcmKeyframeSetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::KeyframeSet_t* msg)
{
	std::cout << "keyframes received: " << msg->kf_num << std::endl;

	// copy keyframes from msg
	KeyframeSet new_kfs;
	for(int i = 0; i < msg->kf_num; i++)
	{
		Keyframe kf;

		for(int j = 0; j < 3; j++)
		{
			kf.positions[j] = msg->kfs[i].positions[j];
			kf.velocity[j] = msg->kfs[i].velocity[j];
		}
		kf.vel_constr = msg->kfs[i].vel_constr;
		kf.yaw = msg->kfs[i].yaw;

		new_kfs.keyframes.push_back(kf);
	}

	GenerateTrajectory(new_kfs);
}

void TrajectoryGenerator::GenerateTrajectory(KeyframeSet& kfs)
{
	srcl_lcm_msgs::PolynomialCurve_t poly_msg;
	uint8_t kf_num = kfs.keyframes.size();

	if(kf_num < 2)
		return;

	poly_msg.wp_num = kfs.keyframes.size();

	//	traj_opt_.InitOptJointMatrices(kf_num);
	traj_opt_.InitOptWithCorridorJointMatrices(kf_num, 20, 0.01);

	for(int i = 0; i < kfs.keyframes.size(); i++)
	{
		traj_opt_.keyframe_x_vals_(0,i) = kfs.keyframes[i].positions[0];
		traj_opt_.keyframe_y_vals_(0,i) = kfs.keyframes[i].positions[1];
		traj_opt_.keyframe_z_vals_(0,i) = kfs.keyframes[i].positions[2];

		if(kfs.keyframes[i].vel_constr)
		{
			traj_opt_.keyframe_x_vals_(1,i) = kfs.keyframes[i].velocity[0];
			traj_opt_.keyframe_y_vals_(1,i) = kfs.keyframes[i].velocity[1];
			traj_opt_.keyframe_z_vals_(1,i) = kfs.keyframes[i].velocity[2];
		}
		else
		{
			traj_opt_.keyframe_x_vals_(1,i) = std::numeric_limits<float>::infinity();
			traj_opt_.keyframe_y_vals_(1,i) = std::numeric_limits<float>::infinity();
			traj_opt_.keyframe_z_vals_(1,i) = std::numeric_limits<float>::infinity();
		}

		traj_opt_.keyframe_x_vals_(2,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(2,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(2,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_x_vals_(3,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(3,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(3,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_yaw_vals_(0,i) = kfs.keyframes[i].yaw;
		traj_opt_.keyframe_yaw_vals_(1,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_ts_(0,i) = i * 0.5;

		srcl_lcm_msgs::WayPoint_t wpoint;
		wpoint.positions[0] = kfs.keyframes[i].positions[0];
		wpoint.positions[1] = kfs.keyframes[i].positions[1];
		wpoint.positions[2] = kfs.keyframes[i].positions[2];
		poly_msg.waypoints.push_back(wpoint);
	}

	traj_opt_.keyframe_x_vals_(1,0) = 0.0;
	traj_opt_.keyframe_y_vals_(1,0) = 0.0;
	traj_opt_.keyframe_z_vals_(1,0) = 0.0;

	traj_opt_.keyframe_x_vals_(2,0) = 0.0;
	traj_opt_.keyframe_y_vals_(2,0) = 0.0;
	traj_opt_.keyframe_z_vals_(2,0) = 0.0;

	traj_opt_.keyframe_x_vals_(3,0) = 0.0;
	traj_opt_.keyframe_y_vals_(3,0) = 0.0;
	traj_opt_.keyframe_z_vals_(3,0) = 0.0;

	traj_opt_.keyframe_yaw_vals_(1,0) = 0;

	traj_opt_.keyframe_x_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(1,kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(2,kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(2,kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(2,kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(3,kf_num - 1) = 0;
	traj_opt_.keyframe_y_vals_(3,kf_num - 1) = 0;
	traj_opt_.keyframe_z_vals_(3,kf_num - 1) = 0;

	traj_opt_.keyframe_yaw_vals_(1,kf_num - 1) = 0;

	//traj_opt_.OptimizeFlatTrajJoint();
	traj_opt_.OptimizeFlatTrajWithCorridorJoint();

	// send results to LCM network
	poly_msg.seg_num = traj_opt_.flat_traj_.traj_segs_.size();
	for(auto& seg : traj_opt_.flat_traj_.traj_segs_)
	{
		srcl_lcm_msgs::PolyCurveSegment_t seg_msg;

		seg_msg.coeffsize_x = seg.seg_x.param_.coeffs.size();
		seg_msg.coeffsize_y = seg.seg_y.param_.coeffs.size();
		seg_msg.coeffsize_z = seg.seg_z.param_.coeffs.size();
		seg_msg.coeffsize_yaw = seg.seg_yaw.param_.coeffs.size();
		for(auto& coeff:seg.seg_x.param_.coeffs)
			seg_msg.coeffs_x.push_back(coeff);
		for(auto& coeff:seg.seg_y.param_.coeffs)
			seg_msg.coeffs_y.push_back(coeff);
		for(auto& coeff:seg.seg_z.param_.coeffs)
			seg_msg.coeffs_z.push_back(coeff);
		for(auto& coeff:seg.seg_yaw.param_.coeffs)
			seg_msg.coeffs_yaw.push_back(coeff);

		seg_msg.t_start = seg.t_start;
		seg_msg.t_end = seg.t_end;

		poly_msg.segments.push_back(seg_msg);
	}

	lcm_->publish("quad_planner/trajectory_polynomial", &poly_msg);
}


