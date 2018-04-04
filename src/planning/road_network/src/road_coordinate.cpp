/* 
 * road_coordinate.cpp
 * 
 * Created on: Apr 03, 2018 18:08
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_network/road_coordinate.hpp"

using namespace librav;

void RoadCoordinate::SetRange(double x_min, double x_max, double y_min, double y_max)
{
    x_min_ = x_min;
    x_max_ = x_max;
    y_min_ = y_min;
    y_max_ = y_max;
}