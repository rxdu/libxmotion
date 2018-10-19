/* 
 * traffic_channel.cpp
 * 
 * Created on: Oct 18, 2018 08:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_map/details/traffic_channel.hpp"

#include <iostream>

#include "road_map/road_map.hpp"

using namespace librav;

TrafficChannel::TrafficChannel(RoadMap *map, std::string src, std::string dst, std::vector<std::string> lanes) : road_map_(map), source_(src), sink_(dst), lanes_(lanes)
{
    for (auto &lane : lanes)
        center_line_ = center_line_.SeriesConcatenate(road_map_->GetLaneCenterLine(lane));
    // std::cout << "channel " << source_ << " -> " << sink_ << " : " << center_line_.GetPointNumer() << std::endl;

    center_curve_ = PathCurve(center_line_);
}
