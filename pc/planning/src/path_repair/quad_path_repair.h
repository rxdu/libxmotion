/*
 * quad_path_repair.h
 *
 *  Created on: Sep 9, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PATH_REPAIR_QUAD_PATH_REPAIR_H_
#define PLANNING_SRC_PATH_REPAIR_QUAD_PATH_REPAIR_H_

#include <memory>
#include "opencv2/opencv.hpp"

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

#include "common/planning_types.h"
#include "common/control_types.h"
#include "planner/graph_planner.h"
#include "map/map_info.h"
#include "geometry/geo_mark.h"
#include "path_repair/graph_combiner.h"
#include "local3d/octomap_server.h"
#include "quad_flat/quad_polyopt.h"
#include "mission/mission_tracker.h"
#include "mission/trajectory_generator.h"

namespace srcl_ctrl {

class QuadPathRepair{
public:
	QuadPathRepair(std::shared_ptr<lcm::LCM> lcm);
	~QuadPathRepair();

private:
	// lcm
	std::shared_ptr<lcm::LCM> lcm_;

	// planners
	GraphPlanner<QuadTree> qtree_planner_;
	GraphPlanner<SquareGrid> sgrid_planner_;
	GraphCombiner<SquareCell*, SquareGrid> gcombiner_;
//	GraphCombiner<QuadTreeNode*, QuadTree> gcombiner_;
	OctomapServer octomap_server_;

	MissionTracker mission_tracker_;
	time_stamp current_sys_time_;

	// trajectory optimization
	QuadPolyOpt traj_opt_;
	std::shared_ptr<TrajectoryGenerator> traj_gen_;

	// planning parameters
	Position2D start_pos_;
	Position2D goal_pos_;

	bool gstart_set_;
	bool ggoal_set_;

	bool world_size_set_;
	bool auto_update_pos_;

	bool init_plan_found_;
	double est_dist2goal_;

public:
	bool update_global_plan_;
	GraphPlannerType active_graph_planner_;

private:
	template<typename PlannerType>
	srcl_lcm_msgs::Graph_t GetLcmGraphFromPlanner(const PlannerType& planner);

public:
	// graph planner configuration
	void ConfigGraphPlanner(MapConfig config, double world_size_x, double world_size_y);

	// general planner configuration
	void EnablePositionAutoUpdate(bool cmd) { auto_update_pos_ = cmd; };

	void SetStartRefWorldPosition(Position2Dd pos);
	void SetGoalRefWorldPosition(Position2Dd pos);

	// search functions
	std::vector<Position2D> UpdateGlobalPath();
	std::vector<uint64_t> UpdateGlobalPathID();

private:
	// lcm
	void LcmTransformHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::QuadrotorTransform* msg);
	void LcmOctomapHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::NewDataReady_t* msg);
	void LcmMissionInfoHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::MissionInfo_t* msg);
	void LcmSysTimeHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::TimeStamp_t* msg);

	// set start and goal on map (internal use)
	void SetStartMapPosition(Position2D pos);
	void SetGoalMapPosition(Position2D pos);

	// helper functions
	cv::Mat GetActiveMap();
	MapInfo GetActiveMapInfo();
	srcl_lcm_msgs::Graph_t GenerateLcmGraphMsg();
	srcl_lcm_msgs::Path_t GenerateLcmPathMsg(std::vector<Position2D> waypoints);
	void Send3DSearchPathToVis(std::vector<Position3Dd> path);
};

}

#endif /* PLANNING_SRC_PATH_REPAIR_QUAD_PATH_REPAIR_H_ */
