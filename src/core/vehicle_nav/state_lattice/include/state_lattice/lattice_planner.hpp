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
#include <tuple>
#include <queue>
#include <functional>
#include <utility>
#include <cmath>
#include <algorithm>
#include <type_traits>
#include <functional>
#include <iostream>
#include <memory>

#include "graph/graph.hpp"
#include "graph/details/priority_queue.hpp"

#include "road_network/road_map.hpp"
#include "state_lattice/lattice_manager.hpp"

namespace librav
{
struct LatticeNode
{
	LatticeNode(double _x, double _y, double _theta)
		: x(_x), y(_y), theta(_theta) { id = (++LatticeNode::instance_count); }
	LatticeNode() { id = (++LatticeNode::instance_count); }

	static int64_t instance_count;

	int64_t id;
	double x = 0;
	double y = 0;
	// double v = 0;
	double theta = 0;

	int64_t GetUniqueID() const { return id; }
};

using LatticePath = std::vector<MotionPrimitive>;

class LatticePlanner
{
  public:
	LatticePlanner() = default;
	LatticePlanner(std::shared_ptr<LatticeManager> lm) : lattice_manager_(lm) {}
	LatticePlanner(std::shared_ptr<LatticeManager> lm, std::shared_ptr<RoadMap> map) : lattice_manager_(lm), road_map_(map) {}

	void SetDrivableAreaMask(std::shared_ptr<DenseGrid> mask) { drivable_mask_ = mask; }

	LatticePath Search(LatticeNode start_state, LatticeNode goal_state);
	LatticePath AStarSearch(LatticeNode start_state, LatticeNode goal_state);

  private:
	double threshold_ = 0.5;
	std::shared_ptr<RoadMap> road_map_;
	std::shared_ptr<DenseGrid> drivable_mask_;
	std::shared_ptr<LatticeManager> lattice_manager_;

	std::vector<std::tuple<LatticeNode, MotionPrimitive>> GenerateLattices(LatticeNode node);
	double CalculateDistance(LatticeNode node0, LatticeNode node1);
	double CalculateHeuristic(LatticeNode node0, LatticeNode node1);
	static std::vector<Vertex_t<LatticeNode, MotionPrimitive> *> ReconstructPath(Vertex_t<LatticeNode, MotionPrimitive> *start_vtx, Vertex_t<LatticeNode, MotionPrimitive> *goal_vtx);
};
} // namespace librav

#endif /* LATTICE_PLANNER_HPP */
