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
    lane_block_footprint_.AddPoint(1.2 * 2, -0.9 * 2);
    lane_block_footprint_.AddPoint(1.2 * 2, 0.9 * 2);
    lane_block_footprint_.AddPoint(-1.2 * 2, 0.9 * 2);
    lane_block_footprint_.AddPoint(-1.2 * 2, -0.9 * 2);

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
                    traffic_regions_.insert(std::make_pair(src_id, src_bk));
                    traffic_regions_.insert(std::make_pair(dst_id, dst_bk));

                    src_bk->center_line = road_map_->GetLaneCenterLine(src_name);
                    dst_bk->center_line = road_map_->GetLaneCenterLine(dst_name);

                    graph_->AddEdge(src_bk, dst_bk, 1.0);
                }
            }
        }
}

TrafficMap::~TrafficMap()
{
    for (auto &entry : traffic_regions_)
        delete entry.second;

    for (auto &entry : traffic_flows_)
        delete entry.second;
}

void TrafficMap::DiscretizeTrafficRegions(double resolution)
{
    /* Form traffic channels */
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

                    std::string prev_block_name;
                    if (path_index == 0)
                        prev_block_name = "none";
                    else
                        prev_block_name = *(it - 1);

                    remainder = DecomposeTrafficRegion(region, prev_block_name, remainder, resolution);
                    fps.insert(fps.end(), region->discrete_lane_blocks[prev_block_name].begin(), region->discrete_lane_blocks[prev_block_name].end());

                    ++path_index;

                    regions.push_back(region);
                    blocks.insert(blocks.end(), region->discrete_lane_blocks[prev_block_name].begin(), region->discrete_lane_blocks[prev_block_name].end());
                }
                traffic_channels_.insert(std::make_pair(std::make_pair(source, sink), TrafficChannel(source, sink, regions, blocks)));
            }
        }
    }

    // std::cout << "total number of channels discretized: " << traffic_channels_.size() << std::endl;

    /* Form traffic flows */
    std::map<std::string, std::vector<TrafficChannel>> chn_mapping;
    for (auto &chn : traffic_channels_)
        chn_mapping[chn.first.first].push_back(chn.second);

    for (auto &entry : chn_mapping)
        traffic_flows_.insert(std::make_pair(entry.first, new TrafficFlow(entry.second)));

    /* Set discretization flag */
    map_discretized_ = true;
}

std::vector<TrafficChannel> TrafficMap::GetAllTrafficChannels()
{
    std::vector<TrafficChannel> chns;
    for (auto &chn : traffic_channels_)
        chns.push_back(chn.second);
    return chns;
}

TrafficFlow *TrafficMap::GetTrafficFlow(std::string source)
{
    auto it = traffic_flows_.find(source);
    if (it != traffic_flows_.end())
        return it->second;
    else
        return nullptr;
}

std::vector<TrafficFlow *> TrafficMap::GetAllTrafficFlows()
{
    std::vector<TrafficFlow *> flows;
    for (auto &flow : traffic_flows_)
        flows.push_back(flow.second);
    return flows;
}

TrafficChannel TrafficMap::GetTrafficChannel(std::string src, std::string dst)
{
    auto check_it = traffic_channels_.find(std::make_pair(src, dst));
    assert(check_it != traffic_channels_.end());
    return check_it->second;
}

std::vector<TrafficChannel> TrafficMap::FindConflictingChannels(std::string src, std::string dst)
{
    std::vector<TrafficChannel> channels;

    if (!map_discretized_)
        return channels;

    auto check_it = traffic_channels_.find(std::make_pair(src, dst));
    if (check_it == traffic_channels_.end())
        return channels;

    TrafficChannel check_chn = check_it->second;
    for (auto &chn : traffic_channels_)
    {
        if (chn.first.first == src && chn.first.second == dst)
            continue;
        if (check_chn.center_line.Intersect(chn.second.center_line))
            channels.push_back(chn.second);
    }
    return channels;
}

std::vector<TrafficFlow *> TrafficMap::FindConflictingFlows(std::string src, std::string dst)
{
    std::vector<TrafficFlow *> flows;

    std::vector<TrafficChannel> channels = FindConflictingChannels(src, dst);

    std::map<std::string, std::vector<TrafficChannel>> chn_mapping;
    for (auto &chn : channels)
        chn_mapping[chn.source].push_back(chn);

    for (auto &entry : chn_mapping)
    {
        if (entry.first != src)
        {
            TrafficFlow *fl = GetTrafficFlow(entry.first);
            if (fl != nullptr)
                flows.push_back(fl);
        }
    }

    return flows;
}

std::vector<TrafficFlow *> TrafficMap::CheckCollision(TrafficFlow *scflow, TrafficFlow *flow)
{
    auto nodes = scflow->CheckSingleChannelCollision(flow);

    std::vector<TrafficFlow *> labeled;
    labeled.push_back(flow);

    return labeled;
}

//////////////////////////////////////////////////////////////////////////////////////////////

double TrafficMap::DecomposeTrafficRegion(TrafficRegion *region, std::string last_region, double last_remainder, double resolution)
{
    double remainder = last_remainder;
    std::vector<Polygon> fps;
    std::vector<VehiclePose> anchors;

    Polyline line = region->center_line;

    // std::cout << "decompose " << region->name << " with remainder " << last_remainder << std::endl;

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
                anchors.push_back(new_pose);
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
            anchors.push_back(new_pose);
        }

        // continue decomposition
        while (accumulated + resolution < segment_len)
        {
            accumulated += resolution;

            VehiclePose new_pose = InterpolatePose(p0, p1, accumulated);
            anchors.push_back(new_pose);
        }
        if (accumulated + resolution == segment_len)
            remainder = 0;
        else
            remainder = segment_len - accumulated;
    }

    // save flags
    region->discretized = true;
    region->remainder = remainder;

    region->discrete_anchor_points.insert(std::make_pair(last_region, anchors));
    region->discrete_remainders.insert(std::make_pair(last_region, remainder));

    for (auto &pt : anchors)
        fps.push_back(lane_block_footprint_.TransformRT(pt.x, pt.y, pt.theta));
    region->discrete_lane_blocks.insert(std::make_pair(last_region, fps));

    // std::cout << " > left  by " << region->name << " : " << remainder << std::endl;

    return remainder;
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

void TrafficMap::LabelConflictBlocks(TrafficFlow *flow)
{
    // UniqueTree<FlowUnit> &ego_tree = flow_tree_;
    // UniqueTree<FlowUnit> &other_tree = flow->flow_tree_;

    // UniqueTree<FlowUnit>::NodeType *node = ego_tree.GetRootNode();

    // std::queue<UniqueTree<FlowUnit>::NodeType *> q;
    // q.push(node);
    // while (!q.empty())
    // {
    //     node = q.front();
    //     q.pop();
    //     // std::cout << node->state << std::endl;
    //     CheckCollision(&other_tree, node->state);
    //     for (auto &nd : node->children)
    //         q.push(nd.first);
    // }
}
