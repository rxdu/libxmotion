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

#include "common/planning_types.h"
#include "common/control_types.h"

#include "planning/geometry/geo_mark.h"
#include "planning/map/map_info.h"

#include "quadrotor/path_repair/sg_graph_planner.h"
#include "quadrotor/path_repair/geo_mark_graph.h"
#include "quadrotor/path_repair/nav_field.h"
#include "quadrotor/path_repair/shortcut_eval.h"
#include "quadrotor/mission/path_manager.h"
#include "quadrotor/mission/mission_tracker.h"

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

	bool map_received_;
	bool update_global_plan_;

  public:	
	// general planner configuration
	void SetStartPosition(Position2D pos);
	void SetGoalPosition(Position2D pos);
	void SetStartHeight(int32_t height);
	void SetGoalHeight(int32_t height);
	void SetMapSize(int32_t x, int32_t y, int32_t z);

	// check if general planner configuration is complete
	bool IsConfigComplete();

	// graph planner configuration
	void SetSensorRange(double meter) { sensor_range_ = meter; };

	// search functions
	std::vector<uint64_t> UpdateGlobal2DPath();

	void RequestNewMap();
	void ResetPlanner();
	SimPath UpdatePath(Position2D pos, int32_t height, double heading);

  private:
	// lcm
	std::shared_ptr<lcm::LCM> lcm_;

	// planners
	SGGraphPlanner sgrid_planner_;
	GeoMarkGraph geomark_graph_;

	SimMapInfo map_info_;
	double sensor_range_;
	
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

	Position2D start_pos_;
	Position2D goal_pos_;
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
	void Send3DSearchPathToVis(Path_t<CubeCell &>& path);
	void SendCubeArrayGraphToVis(std::shared_ptr<Graph_t<CubeCell &>> graph);
};
}

#endif /* PLANNING_SRC_PATH_REPAIR_SIM_PATH_REPAIR_H_ */
