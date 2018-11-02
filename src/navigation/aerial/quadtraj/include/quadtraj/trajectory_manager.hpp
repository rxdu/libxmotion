/* 
 * trajectory_manager.hpp
 * 
 * Created on: Apr 02, 2018 23:57
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAJECTORY_MANAGER_HPP
#define TRAJECTORY_MANAGER_HPP

#include <vector>
#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.hpp"

#include "quadtraj/quad_flattraj.hpp"
#include "quadtraj/quad_polyopt.hpp"

namespace librav
{

class TrajectoryManager
{
  public:
	TrajectoryManager(std::shared_ptr<lcm::LCM> lcm);
	~TrajectoryManager() = default;

	UAVTrajectoryPoint GetCurrentDesiredState(TimeStamp t);

  private:
	std::shared_ptr<lcm::LCM> lcm_;

	bool traj_available_ = false;
	std::vector<Position3Dd> waypoints_;
	QuadFlatTraj active_trajectory_;

	TimeStamp traj_start_time_ = 0;
	double remaining_dist_ = 0;
	int next_wp_idx_ = 0;

	double scaling_factor_ = 1.0;
	int64_t traj_id_ = 0;
	uint64_t user_path_id_ = 0;

	double GetRefactoredTime(double ts, double te, double t);
	double CalcFlightTime(Position3Dd start, Position3Dd goal, double vel);
	void ReportProgress();
	void SendActiveTrajectoryToLCM();
	void GenerateTrajectory(KeyframeSet &kfs, uint64_t traj_id);

	void LcmWaypointsHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const srcl_lcm_msgs::Path_t *msg);
	void LcmKeyframeSetHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const srcl_lcm_msgs::KeyframeSet_t *msg);
};
}

#endif /* TRAJECTORY_MANAGER_HPP */
