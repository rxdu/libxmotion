/* 
 * road_square_grid.hpp
 * 
 * Created on: Apr 12, 2018 17:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROAD_SQUARE_GRID_HPP
#define ROAD_SQUARE_GRID_HPP

#include <memory>
#include <vector>
#include <tuple>
#include <unordered_map>

#include "decomp/square_grid.hpp"
#include "decomp/dense_grid.hpp"

namespace librav
{
struct RoadCellAttributes
{
  double map_cost;
  std::unordered_map<std::string, GridCoordinate> parent_;
  std::unordered_map<std::string, double> cost_;
};

using RoadSquareCell = SquareCellBase<RoadCellAttributes>;
using RoadSquareGrid = SquareGridBase<RoadCellAttributes>;

class GetRoadSquareGridNeighbour
{
public:
  GetRoadSquareGridNeighbour(std::shared_ptr<RoadSquareGrid> sg, std::shared_ptr<DenseGrid> mask = nullptr);

  // define the functor operator
  std::vector<std::tuple<RoadSquareCell *, double>> operator()(RoadSquareCell *cell);

private:
  std::shared_ptr<RoadSquareGrid> sgrid_;
  std::shared_ptr<DenseGrid> mask_;
};
}

#endif /* ROAD_SQUARE_GRID_HPP */
