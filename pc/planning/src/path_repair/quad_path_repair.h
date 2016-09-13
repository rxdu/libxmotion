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
#include "planner/graph_planner.h"
#include "map/map_info.h"
#include "path_repair/geo_mark.h"
#include "path_repair/graph_combiner.h"

namespace srcl_ctrl {

class QuadPathRepair{
public:
	QuadPathRepair();
	QuadPathRepair(std::shared_ptr<lcm::LCM> lcm);
	~QuadPathRepair();

private:
	// lcm
	std::shared_ptr<lcm::LCM> lcm_;

	// planners
	GraphPlanner<QuadTree> qtree_planner_;
	GraphPlanner<SquareGrid> sgrid_planner_;
	GraphCombiner<SquareCell*, SquareGrid> gcombiner_;

	// planning parameters
	Position2D start_pos_;
	Position2D goal_pos_;

	bool gstart_set_;
	bool ggoal_set_;

	bool world_size_set_;
	bool auto_update_pos_;

public:
	bool update_global_plan_;
	GraphPlannerType active_graph_planner_;

private:
	template<typename PlannerType>
	srcl_msgs::Graph_t GetLcmGraphFromPlanner(const PlannerType& planner);

public:
	// graph planner configuration
	void ConfigGraphPlanner(MapConfig config, double world_size_x, double world_size_y);

	// set start and goal
	void SetStartMapPosition(Position2D pos);
	void SetGoalMapPosition(Position2D pos);

	void SetStartRefWorldPosition(Position2Dd pos);
	void SetGoalRefWorldPosition(Position2Dd pos);

	// general planner configuration
	void EnablePositionAutoUpdate(bool cmd) { auto_update_pos_ = cmd; };

	// search functions
	std::vector<Position2D> SearchForGlobalPath();
	std::vector<uint64_t> SearchForGlobalPathID();

	// lcm
	void LcmTransformHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::QuadrotorTransform* msg);

	// helper functions
	cv::Mat GetActiveMap();
	MapInfo GetActiveMapInfo();
	srcl_msgs::Graph_t GenerateLcmGraphMsg();
	srcl_msgs::Path_t GenerateLcmPathMsg(std::vector<Position2D> waypoints);
};

}


#endif /* PLANNING_SRC_PATH_REPAIR_QUAD_PATH_REPAIR_H_ */
