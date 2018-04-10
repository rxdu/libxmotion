/* 
 * map_analysis.hpp
 * 
 * Created on: Apr 05, 2018 18:23
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MAP_ANALYSIS_HPP
#define MAP_ANALYSIS_HPP

#include <memory>
#include <vector>
#include <tuple>
#include <unordered_map>

#include <Eigen/Dense>

#include "decomp/square_grid.hpp"

#include "graph/graph.hpp"
#include "graph/algorithms/dijkstra.hpp"

namespace librav
{
struct RoadCellAttributes
{
  double map_cost;
};

using RoadSquareCell = SquareCellBase<RoadCellAttributes>;
using RoadSquareGrid = SquareGridBase<RoadCellAttributes>;

class GetSquareGridNeighbour
{
public:
  GetSquareGridNeighbour(std::shared_ptr<RoadSquareGrid> sg);

  // define the functor operator
  std::vector<std::tuple<RoadSquareCell *, double>> operator()(RoadSquareCell *cell);

private:
  std::shared_ptr<RoadSquareGrid> sgrid_;
};

////////////////////////////////////////////////////////////////////

namespace MapAnalysis
{
void GenerateGraphCostMap(RoadSquareGrid *grid, Graph_t<RoadSquareCell *> *graph);
};
}

#endif /* MAP_ANALYSIS_HPP */
