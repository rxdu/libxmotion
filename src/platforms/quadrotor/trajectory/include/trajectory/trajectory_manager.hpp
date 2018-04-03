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
#include "common/quad_flattraj.hpp"

namespace librav
{

class TrajectoryManager
{
  public:
	TrajectoryManager(std::shared_ptr<lcm::LCM> lcm, std::string poly_traj_topic = "quad_planner/trajectory_polynomial");
	~TrajectoryManager() = default;

	UAVTrajectoryPoint GetCurrentDesiredState(time_stamp t);

  private:
	std::shared_ptr<lcm::LCM> lcm_;

	bool traj_available_ = false;
	std::vector<Position3Dd> waypoints_;
	time_stamp traj_start_time_ = 0;
	double remaining_dist_ = 0;
	int next_wp_idx_ = 0;
	double scaling_factor_ = 1.0;
	int64_t traj_id_ = 0;

	QuadFlatTraj flat_traj_;

	double GetRefactoredTime(double ts, double te, double t);
	void ReportProgress();

	void LcmPolyTrajMsgHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const srcl_lcm_msgs::PolynomialCurve_t *msg);
};
}

#endif /* TRAJECTORY_MANAGER_HPP */
