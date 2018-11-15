/* 
 * traffic_channel.cpp
 * 
 * Created on: Oct 18, 2018 08:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_map/traffic_channel.hpp"

#include <cmath>
#include <iostream>
#include <algorithm>

#include <eigen3/Eigen/Dense>

using namespace librav;

TrafficChannel::TrafficChannel(std::shared_ptr<RoadMap> map, std::string src, std::string dst, std::vector<std::string> lanes) : road_map_(map), source_(src), sink_(dst), lanes_(lanes)
{
    for (auto &lane : lanes)
        center_line_ = center_line_.SeriesConcatenate(road_map_->GetLaneCenterLine(lane));
    center_curve_ = CurveFitting::FitApproximateLengthCurve(center_line_);

    // default discretization
    DiscretizeChannel(5, 0.74, 5);

    // std::cout << "channel " << source_ << " -> " << sink_ << " : " << center_line_.GetPointNumer() << std::endl;
}

TrafficChannel::TrafficChannel(std::shared_ptr<RoadMap> map, std::string lane) : road_map_(map), source_(lane), sink_(lane), lanes_({lane})
{
    center_line_ = road_map_->GetLaneCenterLine(lane);
    center_curve_ = CurveFitting::FitApproximateLengthCurve(center_line_);
    DiscretizeChannel(5, 0.74, 5);
}

void TrafficChannel::DiscretizeChannel(double step_t, double step_n, int32_t side_num)
{
    grid_ = std::make_shared<CurvilinearGrid>(center_curve_, step_t, step_n, side_num);
}

void TrafficChannel::DiscretizeChannel(double s_offset, double step_t, double step_n, int32_t side_num)
{
    grid_ = std::make_shared<CurvilinearGrid>(center_curve_, step_t, step_n, side_num, s_offset);
}

double TrafficChannel::GetPointLineDistance(SimplePoint ln_pt1, SimplePoint ln_pt2, SimplePoint pt)
{
    double x1 = ln_pt1.x;
    double y1 = ln_pt1.y;

    double x2 = ln_pt2.x;
    double y2 = ln_pt2.y;

    double x0 = pt.x;
    double y0 = pt.y;

    return std::abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
}

CurvilinearGrid::GridPoint TrafficChannel::FindApproximatePoint(SimplePoint pt)
{
    double dist_before_start;
    int32_t start_segment_idx;
    int32_t normal_dist_sign = 1;

    double shortest_dist = std::numeric_limits<double>::max();
    for (int32_t i = 0; i < center_line_.GetPointNumer() - 1; ++i)
    {
        auto start = center_line_.GetPoint(i);
        auto end = center_line_.GetPoint(i + 1);

        Eigen::Vector2d ref_vec(end.x - start.x, end.y - start.y);
        Eigen::Vector2d pt_vec(pt.x - start.x, pt.y - start.y);

        // check if point projects into the line segment
        double dot_product = pt_vec.dot(ref_vec);
        if (!(dot_product >= 0 && dot_product < ref_vec.dot(ref_vec)))
            continue;

        double dist = GetPointLineDistance(start, end, pt);
        if (dist < shortest_dist)
        {
            shortest_dist = dist;
            dist_before_start = pt_vec.dot(ref_vec.normalized());
            start_segment_idx = i;

            // check normal distance sign
            Eigen::Vector3d seg_vec(ref_vec(0), ref_vec(1), 0);
            Eigen::Vector3d dir_vec(pt_vec(0), pt_vec(1), 0);
            Eigen::Vector3d sign_vec = dir_vec.cross(seg_vec);

            if (sign_vec(2) >= 0)
                normal_dist_sign = -1;
        }
    }
    // std::cout << "shortest distance: " << shortest_dist << std::endl;

    double accumulated = 0;
    for (int i = 0; i < start_segment_idx; ++i)
    {
        auto start = center_line_.GetPoint(i);
        auto end = center_line_.GetPoint(i + 1);
        Eigen::Vector2d ref_vec(end.x - start.x, end.y - start.y);

        accumulated += ref_vec.dot(ref_vec.normalized());
    }
    dist_before_start += accumulated;
    // std::cout << "path segment: " << start_segment_idx << " , traveled: " << dist_before_start << std::endl;

    return CurvilinearGrid::GridPoint(dist_before_start, shortest_dist * normal_dist_sign);
}

bool TrafficChannel::CheckInside(SimplePoint pt)
{
    for (auto &lane : lanes_)
    {
        if (road_map_->GetLanePolygon(lane).CheckInside(pt.x, pt.y))
            return true;
    }
    return false;
}

CurvilinearGrid::GridPoint TrafficChannel::ConvertToPathCoordinate(SimplePoint pt)
{
    assert(CheckInside(pt));

    return FindApproximatePoint(pt);
}

SimplePoint TrafficChannel::ConvertToGlobalCoordinate(CurvilinearGrid::GridPoint pt)
{
    return grid_->ConvertToGlobalCoordinate(pt);
}

CurviGridIndex TrafficChannel::GetIndexFromPosition(SimplePoint pt)
{
    auto path_pos = ConvertToPathCoordinate(pt);
    std::cout << "position in path coordinate: " << path_pos.s << " , " << path_pos.delta << std::endl;
    return grid_->GetIndexFromPathCoordinate(path_pos.s, path_pos.delta);
}

void TrafficChannel::PrintInfo()
{
    std::cout << "Traffic channel " << source_ << " -> " << sink_;
    std::cout << " : ";
    for (auto &lane : lanes_)
        std::cout << lane << " ";
    std::cout << std::endl;
}