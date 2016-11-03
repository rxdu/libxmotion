/*
 * mission_tracker.h
 *
 *  Created on: Nov 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_MISSION_MISSION_TRACKER_H_
#define PLANNING_SRC_MISSION_MISSION_TRACKER_H_

#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <limits>
#include <cmath>

#include "eigen3/Eigen/Geometry"

#include "graph/graph.h"

namespace srcl_ctrl {

class MissionTracker {
public:
	MissionTracker():
		mission_started_(false),
		replan_needed_(true),
		remaining_path_length_(std::numeric_limits<double>::infinity()){};
	~MissionTracker(){};

private:
	Path_t<GeoMark> active_path_;
	Position3Dd current_position_;

	uint32_t FindNearestNextWaypoint()
	{
		uint32_t vtx_idx;
		for(vtx_idx = 0; vtx_idx < active_path_.size() - 1; vtx_idx++)
		{
			Position3Dd wp1 = active_path_[vtx_idx]->bundled_data_.position;
			Position3Dd wp2 = active_path_[vtx_idx + 1]->bundled_data_.position;
			Eigen::Vector3d wp1_vec(wp1.x, wp1.y, wp1.z);
			Eigen::Vector3d wp2_vec(wp2.x, wp2.y, wp2.z);
			Eigen::Vector3d test_vec = wp2_vec - wp1_vec;

			Eigen::Vector3d pos_vec(current_position_.x, current_position_.y, current_position_.z);
			Eigen::Vector3d dir_vec = wp1_vec - pos_vec;

			if(dir_vec.dot(test_vec) >= 0)
				break;
		}

		return vtx_idx;
	}

	void UpdateRemainingPathLenght(uint32_t current_idx)
	{
		remaining_path_length_ = 0;
		for(int i = current_idx; i < active_path_.size() - 1; i++)
		{
			Position3Dd pos1 = active_path_[i]->bundled_data_.position;
			Position3Dd pos2 = active_path_[i+1]->bundled_data_.position;
			remaining_path_length_ += std::sqrt(std::pow(pos1.x - pos2.x, 2) +
					std::pow(pos1.y - pos2.y, 2) + std::pow(pos1.z - pos2.z, 2));
		}
		std::cout << "remaining path length: " << remaining_path_length_ << std::endl;
	}

public:
	bool mission_started_;
	bool replan_needed_;

	double remaining_path_length_;

public:
	void UpdateActivePathWaypoints(Path_t<GeoMark>& path) {
		active_path_ = path;
		mission_started_ = true;
		replan_needed_ = true;

		double cost = 0;
		for(int i = 0; i < active_path_.size() - 1; i++)
		{
			Position3Dd pos1 = active_path_[i]->bundled_data_.position;
			Position3Dd pos2 = active_path_[i+1]->bundled_data_.position;
			cost += std::sqrt(std::pow(pos1.x - pos2.x, 2) +
					std::pow(pos1.y - pos2.y, 2) + std::pow(pos1.z - pos2.z, 2));
		}
		std::cout << "init remaining path length: " << cost << std::endl;
	};

	void UpdateCurrentPosition(Position3Dd pos) {
		current_position_ = pos;

		if(mission_started_)
		{
			uint32_t vtx_idx = FindNearestNextWaypoint();
			if(vtx_idx == active_path_.size() - 1)
			{
				replan_needed_ = false;
				remaining_path_length_ = 0;
			}
			else
			{
				UpdateRemainingPathLenght(vtx_idx);
			}
		}
	};

};

}

#endif /* PLANNING_SRC_MISSION_MISSION_TRACKER_H_ */
