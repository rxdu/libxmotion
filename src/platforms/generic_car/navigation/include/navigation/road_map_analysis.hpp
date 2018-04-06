/* 
 * road_map_analysis.hpp
 * 
 * Created on: Apr 05, 2018 18:23
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROAD_MAP_ANALYSIS_HPP
#define ROAD_MAP_ANALYSIS_HPP

#include <memory>
#include <vector>
#include <tuple>

#include <Eigen/Dense>

#include "decomp/square_grid.hpp"

#include "graph/graph.hpp"
#include "graph/algorithms/dijkstra.hpp"

namespace librav
{
class GetSquareGridNeighbour
{
public:
  GetSquareGridNeighbour(std::shared_ptr<SquareGrid> sg);

  // define the functor operator
  std::vector<std::tuple<SquareCell *, double>> operator()(SquareCell *cell);

private:
  std::shared_ptr<SquareGrid> sgrid_;
};
}

#endif /* ROAD_MAP_ANALYSIS_HPP */
