/* 
 * lattice_graph.hpp
 * 
 * Created on: Aug 08, 2018 23:43
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_GRAPH_HPP
#define LATTICE_GRAPH_HPP

#include <memory>
#include <cstdint>

#include "graph/graph.hpp"

#include "state_lattice/lattice_manager.hpp"
#include "state_lattice/lattice_node.hpp"

namespace librav
{
class LatticeGraph
{
public:
  LatticeGraph();

  using GraphType = Graph<LatticeNode, MotionPrimitive>;

  void LoadMotionPrimitives(std::string file);
  void GenerateGraph(int32_t n_unit_time);

  std::shared_ptr<GraphType> GetGraph() { return graph_; }

private:
  bool use_roadmap_ = false;
  std::shared_ptr<LatticeManager> lattice_manager_;
  std::shared_ptr<GraphType> graph_;
  

  std::vector<std::tuple<LatticeNode, MotionPrimitive>> GenerateLattices(LatticeNode node);
};
} // namespace librav

#endif /* LATTICE_GRAPH_HPP */
