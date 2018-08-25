/* 
 * traffic_flow_map.cpp
 * 
 * Created on: Aug 20, 2018 22:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_map/traffic_map.hpp"

#include <cmath>
#include <cassert>
#include <algorithm>

using namespace librav;

TrafficMap::TrafficMap(std::shared_ptr<RoadMap> map) : road_map_(map)
{
    lane_block_footprint_.AddPoint(1.2 * 2, 0.9 * 2);
    lane_block_footprint_.AddPoint(1.2 * 2, -0.9 * 2);
    lane_block_footprint_.AddPoint(-1.2 * 2, -0.9 * 2);
    lane_block_footprint_.AddPoint(-1.2 * 2, 0.9 * 2);

    ConstructLaneGraph();
}

TrafficMap::~TrafficMap()
{
    for (auto &entry : flow_regions_)
        delete entry.second;
}

void TrafficMap::ConstructLaneGraph()
{
    graph_ = std::make_shared<Graph_t<TrafficRegion *>>();

    for (auto &source : road_map_->GetSources())
        for (auto &sink : road_map_->GetSinks())
        {
            auto path = road_map_->FindShortestRoute(source, sink);
            if (!path.empty())
            {
                for (auto it = path.begin(); it != path.end() - 1; ++it)
                {
                    int32_t src_id = *it;
                    int32_t dst_id = *(it + 1);

                    std::string src_name = road_map_->GetLaneletNameFromID(src_id);
                    std::string dst_name = road_map_->GetLaneletNameFromID(dst_id);

                    TrafficRegion *src_bk = new TrafficRegion(src_id, src_name);
                    TrafficRegion *dst_bk = new TrafficRegion(dst_id, dst_name);
                    flow_regions_.insert(std::make_pair(src_id, src_bk));
                    flow_regions_.insert(std::make_pair(dst_id, dst_bk));

                    src_bk->center_line = road_map_->GetLaneCenterLine(src_name);
                    dst_bk->center_line = road_map_->GetLaneCenterLine(dst_name);

                    graph_->AddEdge(src_bk, dst_bk, 1.0);
                }
            }
        }
}

std::vector<Polygon> TrafficMap::DiscretizeRoadNetwork(double resolution)
{
    std::vector<Polygon> fps;
    for (auto &source : road_map_->GetSources())
    {
        // std::string source = "s2";
        for (auto &sink : road_map_->GetSinks())
        // std::string sink = "s1";
        {
            auto path = road_map_->FindShortestRouteName(source, sink);
            if (!path.empty())
            {
                std::vector<TrafficRegion *> regions;
                std::vector<Polygon> blocks;
                // std::cout << source << " ---> " << sink << std::endl;

                int32_t path_index = 0;
                double remainder = 0;
                for (auto it = path.begin(); it != path.end(); ++it)
                {
                    auto region = graph_->GetVertex(road_map_->GetLaneletIDFromName(*it))->state_;

                    if (path_index == 0)
                        remainder = 0;
                    else
                        remainder = graph_->GetVertex(road_map_->GetLaneletIDFromName(*(it - 1)))->state_->remainder;

                    if (!region->discretized)
                    {
                        // std::cout << "decompose with remainder: " << remainder << std::endl;
                        auto lfp = DecomposeTrafficRegion(region, remainder, resolution);
                        fps.insert(fps.end(), lfp.begin(), lfp.end());
                    }

                    ++path_index;

                    regions.push_back(region);
                    blocks.insert(blocks.end(), region->lane_blocks.begin(), region->lane_blocks.end());
                }
                flow_channels_.insert(std::make_pair(std::make_pair(source, sink), TrafficChannel(source, sink, regions, blocks)));
            }
        }
    }

    // std::cout << "total number of channels discretized: " << flow_channels_.size() << std::endl;
    map_discretized_ = true;

    return fps;
}

TrafficChannel TrafficMap::GetTrafficChannel(std::string src, std::string dst)
{
    auto check_it = flow_channels_.find(std::make_pair(src, dst));
    assert(check_it != flow_channels_.end());
    return check_it->second;
}

std::vector<TrafficChannel> TrafficMap::FindConflictingChannels(std::string src, std::string dst)
{
    std::vector<TrafficChannel> channels;

    if (!map_discretized_)
        return channels;

    auto check_it = flow_channels_.find(std::make_pair(src, dst));
    if (check_it == flow_channels_.end())
        return channels;

    TrafficChannel check_chn = check_it->second;
    for (auto &chn : flow_channels_)
    {
        if (chn.first.first == src && chn.first.second == dst)
            continue;
        if (check_chn.center_line.Intersect(chn.second.center_line))
            channels.push_back(chn.second);
    }
    return channels;
}

std::vector<TrafficFlow> TrafficMap::GetConflictingFlows(std::string src, std::string dst)
{
    std::vector<TrafficFlow> flows;

    std::vector<TrafficChannel> channels = FindConflictingChannels(src, dst);

    std::map<std::string, std::vector<TrafficChannel>> chn_mapping;
    for (auto &chn : channels)
        chn_mapping[chn.source].push_back(chn);

    for (auto &entry : chn_mapping)
    {
        if (entry.first != src)
            flows.push_back(TrafficFlow(entry.second));
    }

    return flows;
}

std::vector<Polygon> TrafficMap::DecomposeTrafficRegion(TrafficRegion *region, double last_remainder, double resolution)
{
    double remainder = last_remainder;
    std::vector<Polygon> fps;

    Polyline line = region->center_line;

    for (int i = 0; i < line.GetPointNumer() - 1; ++i)
    {
        auto p0 = line.GetPoint(i);
        auto p1 = line.GetPoint(i + 1);

        Eigen::Vector2d ref_vec(p1.x - p0.x, p1.y - p0.y);
        double segment_len = ref_vec.dot(ref_vec.normalized());

        double accumulated = 0;
        // deal with remainder first
        // 1. if previous segment has a small amount of distance remaining (<resolution)
        if (remainder > 0)
        {
            accumulated = -remainder;

            if (accumulated + resolution > segment_len)
            {
                remainder = -(accumulated + resolution - segment_len);
                continue;
            }
        }
        // 2. if previous segment is shorter than even one step forward
        else if (remainder < 0)
        {
            accumulated = -remainder;
            if (accumulated < segment_len)
            {
                VehiclePose new_pose = InterpolatePose(p0, p1, accumulated);
                region->anchor_points.push_back(new_pose);
            }
            else
            {
                if (accumulated == segment_len)
                    remainder = 0;
                else
                    remainder = remainder + segment_len;
                continue;
            }
        }
        // 3. add starting point if remainder = 0
        else
        {
            VehiclePose new_pose = InterpolatePose(p0, p1, 0);
            region->anchor_points.push_back(new_pose);
        }

        // continue decomposition
        while (accumulated + resolution < segment_len)
        {
            accumulated += resolution;

            VehiclePose new_pose = InterpolatePose(p0, p1, accumulated);
            region->anchor_points.push_back(new_pose);
        }
        if (accumulated + resolution == segment_len)
            remainder = 0;
        else
            remainder = segment_len - accumulated;
    }

    // save flags
    region->discretized = true;
    region->remainder = remainder;

    // std::cout << " > left: " << remainder << std::endl;

    for (auto &pt : region->anchor_points)
        fps.push_back(lane_block_footprint_.TransformRT(pt.x, pt.y, pt.theta));

    region->lane_blocks = fps;

    return fps;
}

VehiclePose TrafficMap::InterpolatePose(SimplePoint pt0, SimplePoint pt1, double s)
{
    Eigen::Vector2d base(pt1.x - pt0.x, pt1.y - pt0.y);
    double base_len = base.dot(base.normalized());
    Eigen::Vector2d ps = s / base_len * base;
    // std::cout << "s/base_len: " << s / base_len << std::endl;
    Eigen::Vector2d dir = base.normalized();

    Eigen::Vector2d position = Eigen::Vector2d(pt0.x, pt0.y) + ps;
    double yaw = std::atan2(dir(1), dir(0));

    return VehiclePose(position(0), position(1), yaw);
}

// Direction inversed version
VehiclePose TrafficMap::InterpolatePoseInversed(SimplePoint pt0, SimplePoint pt1, double s)
{
    Eigen::Vector2d base(pt1.x - pt0.x, pt1.y - pt0.y);
    double base_len = base.dot(base.normalized());
    Eigen::Vector2d ps = s / base_len * base;
    Eigen::Vector2d dir = -base.normalized();

    Eigen::Vector2d position = Eigen::Vector2d(pt0.x, pt0.y) + ps;
    double yaw = std::atan2(dir(1), dir(0));

    return VehiclePose(position(0), position(1), yaw);
}