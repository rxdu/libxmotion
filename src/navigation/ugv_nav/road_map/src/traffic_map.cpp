/* 
 * traffic_map.cpp
 * 
 * Created on: Oct 18, 2018 08:37
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_map/traffic_map.hpp"

#include "road_map/road_map.hpp"

using namespace librav;

TrafficMap::TrafficMap(RoadMap *map) : road_map_(map)
{
}

void TrafficMap::IdentifyTrafficElements()
{
    auto sinks = road_map_->GetSinks();
}