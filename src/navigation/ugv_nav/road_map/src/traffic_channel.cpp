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

#include <eigen3/Eigen/Dense>

#include "road_map/road_map.hpp"

using namespace librav;

TrafficChannel::TrafficChannel(RoadMap *map, std::string src, std::string dst, std::vector<std::string> lanes) : road_map_(map), source_(src), sink_(dst), lanes_(lanes)
{
    for (auto &lane : lanes)
        center_line_ = center_line_.SeriesConcatenate(road_map_->GetLaneCenterLine(lane));
    center_curve_ = CurveFitting::FitApproximateLengthCurve(center_line_);
    // std::cout << "channel " << source_ << " -> " << sink_ << " : " << center_line_.GetPointNumer() << std::endl;
}

PathCoordinate TrafficChannel::ConvertToPathCoordinate(SimplePoint pt)
{
    // find "s" first

    // find "delta"
}

SimplePoint TrafficChannel::ConvertToGlobalCoordinate(PathCoordinate pt)
{
    double s = origin_offset_ + pt.s;

    auto gpt = center_curve_.Evaluate(s);
    auto vel_vec = center_curve_.Evaluate(s, 1);

    // Eigen::Vec2d 

}

void TrafficChannel::DiscretizeChannel(double step_t, double step_n)
{
    // s-delta coordinate frame
    // generate knots along centerline
    for (double s = 0; s <= center_curve_.GetTotalLength(); s += step_t)
    {
    }
}
