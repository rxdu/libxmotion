/* 
 * left_right_turn.hpp
 * 
 * Created on: Mar 07, 2018 16:42
 * Description: turn left or right at a T-shape intersection 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LEFT_RIGHT_TURN_HPP
#define LEFT_RIGHT_TURN_HPP

#include <memory>

#include "graph/graph.hpp"
#include "geometry/square_grid.hpp"
#include "planner/graph_builder.hpp"

namespace librav
{
namespace LeftRightTurnScenario
{
    std::shared_ptr<SquareGrid> CreateSqaureGrid();
}
}

#endif /* LEFT_RIGHT_TURN_HPP */
