/* 
 * left_right_turn.cpp
 * 
 * Created on: Mar 07, 2018 16:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "scenario/left_right_turn.hpp"

using namespace librav;

std::shared_ptr<SquareGrid> LeftRightTurnScenario::CreateSqaureGrid()
{
    return std::make_shared<SquareGrid>(222,500);
}