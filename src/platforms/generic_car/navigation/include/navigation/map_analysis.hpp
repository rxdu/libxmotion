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

#include <Eigen/Dense>

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"

#include "navigation/road_square_grid.hpp"

namespace librav
{
namespace MapAnalysis
{
void GenerateGraphCostMap(RoadSquareGrid *grid, Graph_t<RoadSquareCell *> *graph);
};
}

#endif /* MAP_ANALYSIS_HPP */
