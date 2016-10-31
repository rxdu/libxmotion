/*
 * waypoint_manager.cpp
 *
 *  Created on: Oct 31, 2016
 *      Author: rdu
 */

#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "path_repair/waypoint_manager.h"

using namespace srcl_ctrl;

WaypointManager::WaypointManager(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm)
{
	lcm_->subscribe("quad_planner/geo_mark_graph_path",&WaypointManager::LcmWaypointsHandler, this);
}

WaypointManager::~WaypointManager()
{

}

std::vector<Position3Dd> WaypointManager::WaypointSelector(std::vector<Position3Dd>& wps)
{
	std::vector<Position3Dd> smoothed_points;
	//int check_point = 0;

	// first remove undesired points at the connections between 2d/3d geomarks
	for(auto it = wps.begin(); it != wps.end() - 1; ++it)
	{
		Position3Dd pt1 = *it;
		Position3Dd pt2 = *(it+1);

		smoothed_points.push_back(pt1);

		// fix the connections between 2d and 3d vertices
		if(std::abs(pt1.z - pt2.z) > 0.05)
		{
//			check_point++;
//			std::cout << "point 1: " << pt1.x << " , " << pt1.y << " , " << pt1.z << std::endl;
//			std::cout << "point 2: " << pt2.x << " , " << pt2.y << " , " << pt2.z << std::endl;
//			std::cout << " ------------------- " << std::endl;

			// ignore the last point if there is a 2d/3d connection
			//	between the last two points
			if(it + 2 != wps.end())
			{
				Position3Dd pt3 = *(it + 2);

				Eigen::Vector3d v1(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
				Eigen::Vector3d v2(pt3.x - pt2.x, pt3.y - pt2.y, pt3.z - pt2.z);

				if(v1.dot(v2) <= 0)
				{
//					std::cout << "point skipped" << std::endl;
					it++;
				}
			}
		}
		else if(it + 2 == wps.end())
			smoothed_points.push_back(pt2);
	}

//	std::cout << "check point num: " << check_point << std::endl;
//	std::cout << "selected points: " << selected.size() << std::endl;
//	std::cout << " ********************** " << std::endl;

	// then remove intermediate points in a staight line
	std::vector<Position3Dd> minimum_points;

	if(smoothed_points.size() <= 2)
	{
		minimum_points = smoothed_points;
	}
	else
	{
		int start_wp_idx = 0;

		// add first waypoint
		minimum_points.push_back(smoothed_points.front());
		// check intermediate waypoints
		while(start_wp_idx < smoothed_points.size() - 1)
		{
			Position3Dd pt1 = smoothed_points[start_wp_idx];

			for(int id = start_wp_idx + 1; id < smoothed_points.size() - 1; ++id)
			{
				Position3Dd pt2 = smoothed_points[id];
				Position3Dd pt3 = smoothed_points[id+1];

				Eigen::Vector3d v1(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
				Eigen::Vector3d v2(pt3.x - pt2.x, pt3.y - pt2.y, pt3.z - pt2.z);

				if(v1.cross(v2).norm() != 0)
				{
					minimum_points.push_back(smoothed_points[id]);
					start_wp_idx = id;
					break;
				}
			}
		}
		// add last waypoint
		minimum_points.push_back(smoothed_points.back());
	}

	std::cout << "minimum waypoints: " << minimum_points.size() << std::endl;

	srcl_msgs::Path_t path_msg;

	path_msg.waypoint_num = minimum_points.size();
	for(auto& wp : minimum_points)
	{
		srcl_msgs::WayPoint_t waypoint;
		waypoint.positions[0] = wp.x;
		waypoint.positions[1] = wp.y;
		waypoint.positions[2] = wp.z;

		path_msg.waypoints.push_back(waypoint);
	}

	lcm_->publish("quad_planner/geomark_wp_selected", &path_msg);

	return smoothed_points;
}

void WaypointManager::LcmWaypointsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::Path_t* msg)
{
	std::cout << "waypoints received: " << msg->waypoint_num << std::endl;

	std::vector<Position3Dd> waypoints;

	for(int i = 0; i < msg->waypoint_num; i++)
		waypoints.push_back(Position3Dd(msg->waypoints[i].positions[0],msg->waypoints[i].positions[1],msg->waypoints[i].positions[2]));

	std::vector<Position3Dd> opt_wps = WaypointSelector(waypoints);
	//CalcTrajFromWaypoints(opt_wps);
}

void WaypointManager::CalcTrajFromWaypoints(std::vector<Position3Dd>& wps)
{
	//std::vector<Position3Dd> opt_wps = WaypointSelector(wps);

	uint8_t kf_num = wps.size();

	if(kf_num < 2)
		return;

	//	traj_opt_.InitOptJointMatrices(kf_num);
	traj_opt_.InitOptWithCorridorJointMatrices(kf_num, 20, 0.01);

	for(int i = 0; i < wps.size(); i++)
	{
		traj_opt_.keyframe_x_vals_(0,i) = wps[i].x;
		traj_opt_.keyframe_y_vals_(0,i) = wps[i].y;
		traj_opt_.keyframe_z_vals_(0,i) = wps[i].z;
		//traj_opt_.keyframe_yaw_vals_(0,i) = 0;

		traj_opt_.keyframe_x_vals_(1,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(1,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(1,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_x_vals_(2,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(2,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(2,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_x_vals_(3,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(3,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(3,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_ts_(0,i) = i * 0.5;
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

	traj_opt_.keyframe_x_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(1,kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(2,kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(2,kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(2,kf_num - 1) = 0.0;

	traj_opt_.keyframe_x_vals_(3,kf_num - 1) = 0;
	traj_opt_.keyframe_y_vals_(3,kf_num - 1) = 0;
	traj_opt_.keyframe_z_vals_(3,kf_num - 1) = 0;

	//traj_opt_.OptimizeFlatTrajJoint();
	traj_opt_.OptimizeFlatTrajWithCorridorJoint();

	// send results to LCM network
	srcl_msgs::Path_t path_msg;

	path_msg.waypoint_num = wps.size();
	for(auto& wp : wps)
	{
		srcl_msgs::WayPoint_t waypoint;
		waypoint.positions[0] = wp.x;
		waypoint.positions[1] = wp.y;
		waypoint.positions[2] = wp.z;

		path_msg.waypoints.push_back(waypoint);
	}

	lcm_->publish("quad_planner/geomark_wp_selected", &path_msg);

	srcl_msgs::PolynomialCurve_t poly_msg;

	poly_msg.seg_num = traj_opt_.flat_traj_.traj_segs_.size();
	for(auto& seg : traj_opt_.flat_traj_.traj_segs_)
	{
		srcl_msgs::PolyCurveSegment_t seg_msg;

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


