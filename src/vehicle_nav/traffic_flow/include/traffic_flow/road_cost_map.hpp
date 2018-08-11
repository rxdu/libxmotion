/* 
 * road_cost_map.hpp
 * 
 * Created on: Apr 05, 2018 18:23
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROAD_COST_MAP_HPP
#define ROAD_COST_MAP_HPP

#include <Eigen/Dense>

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"

#include "traffic_flow/road_square_grid.hpp"

namespace librav
{
namespace RoadCostMap
{
void GenerateGraphCostMap(RoadSquareGrid *grid, Graph_t<RoadSquareCell *> *graph);
void GenerateGraphCostMap(RoadSquareGrid *grid, std::string channel);
void GenerateTrafficDensityCostMap(RoadSquareGrid *grid);
};
}

#endif /* ROAD_COST_MAP_HPP */
