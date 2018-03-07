/*
 * sim_path_repair.h
 *
 *  Created on: Sep 9, 2017
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PATH_REPAIR_SIM_PATH_REPAIR_H_
#define PLANNING_SRC_PATH_REPAIR_SIM_PATH_REPAIR_H_

#include <memory>
#include "opencv2/opencv.hpp"

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.hpp"
#include "common/librav_types.hpp"

#include "geometry/geo_mark.h"
// #include "map/map_info.h"

#include "path_repair/sim/sim_depth_sensor.h"
#include "path_repair/sg_graph_planner.h"
#include "path_repair/nav_field.h"
#include "path_repair/shortcut_eval.h"
#include "mission/mission_tracker.h"

namespace librav
{

struct SimMapInfo
{
	uint32_t size_x;
	uint32_t size_y;
	uint32_t size_z;
	double side_size;
};

struct SimWaypoint
{
	int32_t x;
	int32_t y;
	int32_t z;
	double yaw;

	int32_t id;
};

using SimPath = std::vector<SimWaypoint>;

class SimPathRepair
{
  public:
	SimPathRepair(std::shared_ptr<lcm::LCM> lcm);
	SimPathRepair(std::shared_ptr<lcm::LCM> lcm, std::shared_ptr<SimDepthSensor> dsensor);

	bool map_received_;
	bool update_global_plan_;

  public:
	// general planner configuration
	void SetStartPosition(Position2Di pos);
	void SetGoalPosition(Position2Di pos);
	void SetStartHeight(int32_t height);
	void SetGoalHeight(int32_t height);
	void SetMapSize(int32_t x, int32_t y, int32_t z);
	void SaveMap(std::string map_name);

	// check if general planner configuration is complete
	bool IsConfigComplete();

	// graph planner configuration
	void SetSensorRange(int32_t rng);

	// search functions
	std::vector<uint64_t> UpdateGlobal2DPath();
	double GetGlobal2DPathCost() { return path_2d_cost_; };
	double GetGlobal3DPathCost() { return path_3d_cost_; };

	void RequestNewMap();
	void ResetPlanner();
	SimPath UpdatePath(Position2Di pos, int32_t height, double heading, bool enable_path_repair = true);

  private:
	// lcm
	std::shared_ptr<lcm::LCM> lcm_;
	std::shared_ptr<SimDepthSensor> depth_sensor_;

	// planners
	SGGraphPlanner sgrid_planner_;

	SimMapInfo map_info_;
	int32_t sensor_range_;

	double path_2d_cost_;
	double path_3d_cost_;

	std::shared_ptr<SquareGrid> sgrid_;
	std::shared_ptr<CubeArray> carray_base_;

	std::shared_ptr<NavField<SquareCell *>> nav_field_;
	std::shared_ptr<ShortcutEval> sc_evaluator_;

	std::unique_ptr<MissionTracker> mission_tracker_;

	// planning parameters
	bool pstart_set_;
	bool pgoal_set_;
	bool hstart_set_;
	bool hgoal_set_;

	Position2Di start_pos_;
	Position2Di goal_pos_;
	int32_t start_height_;
	int32_t goal_height_;

	int32_t map_size_[3];

	double est_new_dist_; // temporary calculation result, internal use only

  private:
	// planner
	bool EvaluateNewPath(std::vector<Position3Dd> &new_path);

	// lcm
	void LcmSimMapHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::Map_t *msg);

	// helper functions
	void Send3DSearchPathToVis(Path_t<CubeCell &> &path);
	void SendCubeArrayGraphToVis(std::shared_ptr<Graph_t<CubeCell &>> graph);
};
}

#endif /* PLANNING_SRC_PATH_REPAIR_SIM_PATH_REPAIR_H_ */
