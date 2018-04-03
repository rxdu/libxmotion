/* 
 * polyline.cpp
 * 
 * Created on: Apr 03, 2018 16:05
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_network/polyline.hpp"

#include <iostream>

using namespace librav;

void PolyLine::AddPoint(double x, double y)
{
    points_.insert(std::make_pair(point_num_++, PolyLinePoint(x, y)));
}

void PolyLine::PrintPoints() const
{
    for (auto &pt : points_)
        std::cout << pt.second.x << " , " << pt.second.y << std::endl;
}