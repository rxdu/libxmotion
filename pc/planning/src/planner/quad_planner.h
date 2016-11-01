/*
 * quad_planner.h
 *
 *  Created on: Aug 1, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PLANNER_QUAD_PLANNER_H_
#define PLANNING_SRC_PLANNER_QUAD_PLANNER_H_

#include <memory>
#include "opencv2/opencv.hpp"

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

#include "common/planning_types.h"
#include "planner/graph_planner.h"
#include "planner/rrts_planner.h"
#include "map/map_info.h"

namespace srcl_ctrl {

class QuadPlanner{
public:
	QuadPlanner();
	QuadPlanner(std::shared_ptr<lcm::LCM> lcm);
	~QuadPlanner();

private:
	// lcm
	std::shared_ptr<lcm::LCM> lcm_;

	// planners
	GraphPlanner<QuadTree> qtree_planner_;
	GraphPlanner<SquareGrid> sgrid_planner_;

	RRTStarPlanner local_planner_;

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
	srcl_lcm_msgs::Graph_t GetLcmGraphFromPlanner(const PlannerType& planner);

public:
	// graph planner configuration
	void ConfigGraphPlanner(MapConfig config);

	// set start and goal
	void SetStartMapPosition(Position2D pos);
	void SetGoalMapPosition(Position2D pos);

	void SetStartRefWorldPosition(Position2Dd pos);
	void SetGoalRefWorldPosition(Position2Dd pos);

	// RRT* configuration
	void ConfigRRTSOccupancyMap(cv::Mat map, MapInfo info);
	void SetRealWorldSize(double x, double y);

	// general planner configuration
	void EnablePositionAutoUpdate(bool cmd) { auto_update_pos_ = cmd; };

	// search functions
	std::vector<Position2D> SearchForGlobalPath();
	std::vector<uint64_t> SearchForGlobalPathID();
	bool SearchForLocalPath(Position2Dd start, Position2Dd goal, double time_limit, std::vector<Position2Dd>& path2d);

	// lcm
	void LcmTransformHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_lcm_msgs::QuadrotorTransform* msg);

	// helper functions
	cv::Mat GetActiveMap();
	MapInfo GetActiveMapInfo();
	std::shared_ptr<Graph_t<RRTNode>> GetLocalPlannerVisGraph();
	srcl_lcm_msgs::Graph_t GenerateLcmGraphMsg();
	srcl_lcm_msgs::Path_t GenerateLcmPathMsg(std::vector<Position2D> waypoints);
};

}


#endif /* PLANNING_SRC_PLANNER_QUAD_PLANNER_H_ */
