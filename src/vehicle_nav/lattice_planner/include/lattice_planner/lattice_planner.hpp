/* 
 * lattice_planner.hpp
 * 
 * Created on: Aug 07, 2018 09:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_PLANNER_HPP
#define LATTICE_PLANNER_HPP

#include <vector>
#include <memory>
#include <cstdint>

#include "graph/graph.hpp"

#include "road_map/road_map.hpp"
#include "state_lattice/lattice_manager.hpp"
#include "state_lattice/lattice_node.hpp"

#include "stopwatch/stopwatch.h"

namespace librav
{
using LatticePath = std::vector<MotionPrimitive>;

class LatticePlanner
{
  public:
	LatticePlanner() = delete;
	LatticePlanner(std::shared_ptr<LatticeManager> lm) : lattice_manager_(lm) {}
	LatticePlanner(std::shared_ptr<LatticeManager> lm, std::shared_ptr<RoadMap> map) : lattice_manager_(lm), road_map_(map) {}

	void SetVehicleFootprint(const Polygon &polygon) { footprint_ = polygon; }
	void SetEgoPlanningRoute(std::vector<std::string> ll);

	LatticePath Search(LatticeNode start_state, int32_t horizon);
	LatticePath AStarSearch(LatticeNode start_state, LatticeNode goal_state, int32_t horizon, int32_t min_candidate = 20);

	std::vector<Polyline> ConvertPathToPolyline(const LatticePath &path);

  private:
	double threshold_ = 1.0;
	stopwatch::StopWatch timer_;

	// vehicle footprint
	Polygon footprint_;

	std::shared_ptr<RoadMap> road_map_;
	std::shared_ptr<LatticeManager> lattice_manager_;

	std::vector<std::string> drivable_lanelets_;
	std::vector<Polygon> drivable_polygons_;

	bool VehicleInsideDrivableArea(const Polygon &footprint);
	bool IsCollisionFree(const MotionPrimitive &lattice);

	double CalculateDistance(LatticeNode node0, LatticeNode node1);
	double CalculateHeuristic(LatticeNode node0, LatticeNode node1);
	double EvaluateCandidate(LatticeNode candidate, LatticeNode desired);

	std::vector<std::tuple<LatticeNode, MotionPrimitive>> GenerateLattices(LatticeNode node);
	static std::vector<Vertex_t<LatticeNode, MotionPrimitive> *> ReconstructPath(Vertex_t<LatticeNode, MotionPrimitive> *start_vtx, Vertex_t<LatticeNode, MotionPrimitive> *goal_vtx);
};
} // namespace librav

#endif /* LATTICE_PLANNER_HPP */
